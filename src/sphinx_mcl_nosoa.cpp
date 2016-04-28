// code adapted from continuous.c

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <sys/select.h>

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include <pocketsphinx.h>

//include for ROS publisher
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

static ps_decoder_t *ps;
static cmd_ln_t *config;

/* Sleep for specified msec */
static void sleep_msec(int32 ms)
{
	struct timeval tmo;

	tmo.tv_sec = 0;
	tmo.tv_usec = ms * 1000;

	select(0, NULL, NULL, NULL, &tmo);
}

/*
 * Main utterance processing loop:
 *     for (;;) {
 *        start utterance and wait for speech to process
 *        decoding till end-of-utterance silence will be detected
 *        print utterance result;
 *     }
 */
static void recognize_from_microphone()
{
	ad_rec_t *ad;
	int16 adbuf[2048];
	uint8 utt_started, in_speech;
	int32 k;
	char const *hyp;

	if ((ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"),
					(int) cmd_ln_float32_r(config,
						"-samprate"))) == NULL)
		E_FATAL("Failed to open audio device\n");
	if (ad_start_rec(ad) < 0)
		E_FATAL("Failed to start recording\n");

	if (ps_start_utt(ps) < 0)
		E_FATAL("Failed to start utterance\n");
	utt_started = FALSE;

	//initialize ROS publisher

	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::String>("cmds_received_nosoa", 1000);
	ros::Rate loop_rate(10);

	printf("READY....\n");


	while (ros::ok()) {
		if ((k = ad_read(ad, adbuf, 2048)) < 0)
			E_FATAL("Failed to read audio\n");
		ps_process_raw(ps, adbuf, k, FALSE, FALSE);
		in_speech = ps_get_in_speech(ps);
		if (in_speech && !utt_started) {
			utt_started = TRUE;
			printf("Listening...\n");
		}
		if (!in_speech && utt_started) {
			/* speech -> silence transition, time to start new utterance  */
			ps_end_utt(ps);
			hyp = ps_get_hyp(ps, NULL );
			if (hyp != NULL){
			  printf("[%f]  sphinx_mcl (nosoa) found and will  publish:  %s\n", ros::Time::now().toSec(), hyp);
				//publish utterance
				std_msgs::String msg;
				std::stringstream ss;
				ss << hyp;
				msg.data = ss.str();
				publisher.publish(msg);
			}

			if (ps_start_utt(ps) < 0)
				E_FATAL("Failed to start utterance\n");
			utt_started = FALSE;
			printf("READY....\n");
		}
		loop_rate.sleep();
		//sleep_msec(100);
	}
	ad_close(ad);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "baxter_sphinx_listener_nosoa");
	std::cout << "Modeldir=" << "\n";
	char const *cfg;
	config = cmd_ln_init(NULL, ps_args(), TRUE,
			"-hmm", MODELDIR "/en-us/en-us",
			"-lm", MODELDIR "/en-us/en-us.lm.dmp",
			"-dict", MODELDIR "/en-us/cmudict-en-us.dict",
			NULL);

	if (config == NULL)
		return 1;

	ps_default_search_args(config);
	ps = ps_init(config);
	if (ps == NULL) {
		cmd_ln_free_r(config);
		return 1;
	}

	E_INFO("%s COMPILED ON: %s, AT: %s\n\n", argv[0], __DATE__, __TIME__);

	ps_set_kws(ps, "keyphrase_search", KEYDIR "/keyphrases.list");
	ps_set_search(ps, "keyphrase_search");
	//ps_start_utt();

	recognize_from_microphone();

	ps_free(ps);
	cmd_ln_free_r(config);

	return 0;
}


