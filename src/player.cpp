#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <jack/jack.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "audio_transporter/Audio.h"

jack_port_t *output_port;
jack_client_t *client;
ros::Subscriber sub;
boost::shared_ptr<ros::NodeHandle> z;
bool is_enabled = false;

jack_default_audio_sample_t *ros2jack_buffer;
unsigned int buffer_size;
unsigned int buffer_r = 0;
unsigned int buffer_w = 0;

std::mutex mtx;

int jack_callback(jack_nframes_t nframes, void *arg)
{
	jack_default_audio_sample_t *out = (jack_default_audio_sample_t *)jack_port_get_buffer(output_port, nframes);

	mtx.lock();
	for (int i = 0; i < nframes; i++)
	{
		out[i] = ros2jack_buffer[buffer_r];
		buffer_r++;
		if (buffer_r >= buffer_size)
			buffer_r = 0;
	}
	mtx.unlock();
	return 0;
}

void jack_shutdown(void *arg)
{
	exit(1);
}

void jackaudioCallback(const audio_transporter::Audio::ConstPtr &msg)
{
	int msg_size = msg->size;

	mtx.lock();
	//outputting only one channel
	for (int i = 0; i < msg_size; i++)
	{
		ros2jack_buffer[buffer_w] = msg->data[i];
		buffer_w++;
		if (buffer_w >= buffer_size)
			buffer_w = 0;
	}
	mtx.unlock();
}

void audio_enable_callback(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data && !is_enabled) {
		printf("[audio-player] Creating ROSJack_Subscriber node...\n");
		sub = z->subscribe("audio", 1000, jackaudioCallback);
		printf("[audio-player] done.\n");
		return;
	}
	if (!msg->data && is_enabled) {
		sub.shutdown();
		return;
	}
}

int main(int argc, char *argv[])
{
	const char *client_name = "jack_player";

	printf("[audio-player] Connecting to Jack Server...\n");
	jack_options_t options = JackNoStartServer;
	jack_status_t status;

	client = jack_client_open(client_name, options, &status);
	if (client == NULL)
	{
		printf("[audio-player] jack_client_open() failed, status = 0x%2.0x\n", status);
		if (status & JackServerFailed)
		{
			printf("[audio-player] Unable to connect to JACK server.\n");
		}
		exit(1);
	}

	if (status & JackNameNotUnique)
	{
		client_name = jack_get_client_name(client);
		printf("[audio-player] Warning: other agent with our name is running, `%s' has been assigned to us.\n", client_name);
	}
	jack_set_process_callback(client, jack_callback, 0);
	jack_on_shutdown(client, jack_shutdown, 0);

	printf("[audio-player] Engine sample rate: %d\n", jack_get_sample_rate(client));

	buffer_size = jack_get_buffer_size(client) * 10;
	ros2jack_buffer = (jack_default_audio_sample_t *)malloc(sizeof(jack_default_audio_sample_t) * buffer_size);

	output_port = jack_port_register(client, "output", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);

	if ((output_port == NULL))
	{
		printf("[audio-player] Could not create agent ports. Have we reached the maximum amount of JACK agent ports?\n");
		exit(1);
	}
	if (jack_activate(client))
	{
		printf("[audio-player] Cannot activate client.");
		exit(1);
	}

	printf("[audio-player] Agent activated.\n");
	printf("[audio-player] Connecting ports...\n");
	const char **serverports_names;
	serverports_names = jack_get_ports(client, NULL, NULL, JackPortIsPhysical | JackPortIsInput);
	if (serverports_names == NULL)
	{
		printf("[audio-player] No available physical output (server input) ports.\n");
		exit(1);
	}
	if (jack_connect(client, jack_port_name(output_port), serverports_names[0]))
	{
		printf("[audio-player] Cannot connect output port.\n");
		exit(1);
	}
	free(serverports_names);

	printf("[audio-player] done.\n");

	printf("[audio-player] Creating ROSJack_Subscriber node...\n");
	ros::init(argc, argv, client_name);
	z.reset(new ros::NodeHandle());
	z->subscribe("audio_enable", 1, audio_enable_callback);		
	printf("[audio-player] done.\n");

	ros::spin();

	jack_client_close(client);
	exit(0);
}
