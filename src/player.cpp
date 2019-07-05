#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <jack/jack.h>
#include "ros/ros.h"
#include "audio_transporter/Audio.h"

jack_port_t *input_port;
jack_client_t *client;

ros::Publisher rosjack_out;

int jack_callback (jack_nframes_t nframes, void *arg){
	jack_default_audio_sample_t *in = (jack_default_audio_sample_t *)jack_port_get_buffer (input_port, nframes);
	
	rosjack::Audio msg;
	msg.size = nframes;
	msg.channels = 1;
	
	msg.data.resize(nframes);
	for (int i = 0; i < nframes; i++){
		msg.data[i] = in[i];
	}
	rosjack_out.publish(msg);
	return 0;
}

void jack_shutdown (void *arg){
	exit (1);
}

int main (int argc, char *argv[]) {	
	const char *client_name = "jack_player";
	
	printf ("Creating ROSJack_Read node...");
	ros::init(argc, argv, client_name);
	ros::NodeHandle n;
	rosjack_out = n.advertise<audio_transporter::Audio>("audio", 1000);
	printf (" done.\n");

	printf ("Connecting to Jack Server...\n");
	jack_options_t options = JackNoStartServer;
	jack_status_t status;
	
	client = jack_client_open (client_name, options, &status);
	if (client == NULL){
		printf ("jack_client_open() failed, status = 0x%2.0x\n", status);
		if (status & JackServerFailed) {
			printf ("Unable to connect to JACK server.\n");
		}
		exit (1);
	}
	
	if (status & JackNameNotUnique){
		client_name = jack_get_client_name(client);
		printf ("Warning: other agent with our name is running, `%s' has been assigned to us.\n", client_name);
	}	
	jack_set_process_callback (client, jack_callback, 0);
	jack_on_shutdown (client, jack_shutdown, 0);	
	printf ("Engine sample rate: %d\n", jack_get_sample_rate (client));
	input_port = jack_port_register (client, "input", JACK_DEFAULT_AUDIO_TYPE,JackPortIsInput, 0);
	if ((input_port == NULL)) {
		printf("Could not create agent ports. Have we reached the maximum amount of JACK agent ports?\n");
		exit (1);
	}	
	if (jack_activate (client)) {
		printf ("Cannot activate client.");
		exit (1);
	}
	
	printf ("Agent activated.\n");	
	printf ("Connecting ports... ");

	const char **serverports_names;
	serverports_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
	if (serverports_names == NULL) {
		printf("No available physical capture (server output) ports.\n");
		exit (1);
	}
	if (jack_connect (client, serverports_names[0], jack_port_name (input_port))) {
		printf("Cannot connect input port.\n");
		exit (1);
	}
	free (serverports_names);	
	printf ("done.\n");
	ros::spin();	
	jack_client_close (client);
	exit (0);
}
