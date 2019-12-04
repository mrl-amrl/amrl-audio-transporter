#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <jack/jack.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "audio_transporter/Audio.h"

jack_port_t *input_port;
jack_client_t *client;
bool is_enabled = false;
const char *source_port;

ros::Publisher rosjack_out;

int jack_callback(jack_nframes_t nframes, void *arg)
{
	if (rosjack_out.getNumSubscribers() == 0)
		return 0;

	jack_default_audio_sample_t *in = (jack_default_audio_sample_t *)jack_port_get_buffer(input_port, nframes);

	audio_transporter::Audio msg;
	msg.size = nframes;
	msg.channels = 1;

	msg.data.resize(nframes);
	for (int i = 0; i < nframes; i++)
		msg.data[i] = in[i];
	rosjack_out.publish(msg);
	return 0;
}

void jack_shutdown(void *arg)
{
	exit(1);
}

void audio_enable_callback(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data && !is_enabled)
	{
		if (jack_connect(client, source_port, jack_port_name(input_port)))
		{
			printf("[audio-sender] Cannot connect output port.\n");
			exit(1);
		}
	}
	else if (!msg->data && is_enabled)
	{
		jack_disconnect(client, source_port, jack_port_name(input_port));
	}
	is_enabled = msg->data;
}

int main(int argc, char *argv[])
{
	const char *client_name = "jack_player";

	ros::init(argc, argv, client_name);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/audio/enable", 1, audio_enable_callback);
	rosjack_out = n.advertise<audio_transporter::Audio>("/audio", 1000);

	printf("[audio-sender] Connecting to Jack Server...\n");
	jack_options_t options = JackNoStartServer;
	jack_status_t status;

	client = jack_client_open(client_name, options, &status);
	if (client == NULL)
	{
		printf("[audio-sender] jack_client_open() failed, status = 0x%2.0x\n", status);
		if (status & JackServerFailed)
		{
			printf("[audio-sender] Unable to connect to JACK server.\n");
		}
		return 1;
	}

	if (status & JackNameNotUnique)
	{
		client_name = jack_get_client_name(client);
		printf("[audio-sender] Warning: other agent with our name is running, `%s' has been assigned to us.\n", client_name);
	}
	jack_set_process_callback(client, jack_callback, 0);
	jack_on_shutdown(client, jack_shutdown, 0);
	printf("[audio-sender] Engine sample rate: %d\n", jack_get_sample_rate(client));
	input_port = jack_port_register(client, "input", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
	if ((input_port == NULL))
	{
		printf("[audio-sender] Could not create agent ports. Have we reached the maximum amount of JACK agent ports?\n");
		return 1;
	}
	if (jack_activate(client))
	{
		printf("[audio-sender] Cannot activate client.");
		return 1;
	}

	printf("[audio-sender] Agent activated.\n");
	const char **serverports_names;
	serverports_names = jack_get_ports(client, NULL, NULL, JackPortIsPhysical | JackPortIsOutput);
	if (serverports_names == NULL)
	{
		printf("[audio-sender] No available physical capture (server output) ports.\n");
		return 1;
	}
	source_port = serverports_names[0];
	free(serverports_names);

	ros::spin();
	jack_client_close(client);
	return 0;
}
