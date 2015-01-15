/*
 * Modified version of main.cpp from tutorial 2, keyboard_control. 
 * This addresses the questions at the end of the tutorial: 
 * 	changing keybindings 
 * 	increasing irat speed
 *	using mouse instead of keyboard
 */

#include <ros/ros.h>
// only need motors for this tutorial
#include <irat_msgs/IRatVelocity.h>
// Use allegro5 for this demo. The tricky part of
// capturing the keyboard is that a window is
// required for focussing the key events.
// Otherwise you would either be stealing key
// presses from other programs (exclusive control),
// or you would be sending spurious key presses to other
// programs (keylogging).
// In this tutorial Allegro5 is used to create a window
// for capturing key events.
#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>
// for I/O
#include <iostream>
#include <fstream>
// for string manipulation
#include <sstream>
// include the header for logging key strokes
#include <keyboard_control/log_key_strokes.h>
// The forward velocity of the irat 
#define VFORWARD 0.2
// Backward velocity 
#define VBACKWARD -0.2

#include <iostream>
#include <stuck/IRatOpticalFlow.h>

using std::cout;
using std::endl;

void flow_callback(stuck::IRatOpticalFlowConstPtr flow) {
    cout << "Got flow callback" << endl;
    if(flow->stuck) {
        cout << "We are stuck. Huzzah!" << endl;
    }
}
/*
 ** Main entry point.
 */
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "keyboard_controller");
	/*
	 ** Initialize allegro and
	 ** install the display and keyboard.
	 */
	if (!al_init() || !al_install_keyboard())
	{
		std::cout << "Could not init Allegron" << std::endl;
		exit(-1);
	}
	// create a display that’s 640x480
	ALLEGRO_DISPLAY * display = al_create_display(640, 480);
	if (!display)
	{
		std::cout << "Could not create display" << std::endl;
		exit(-1);
	}
	// create an event queue
	ALLEGRO_EVENT_QUEUE * event_queue = al_create_event_queue();
	if (!event_queue)
	{
		// not really sure if this is even possible,
		// but it’s in the examples
		// failing to make a queue means the system
		// must be in serious trouble...
		std::cout << "Could not create event queuen" << std::endl;
		exit(-1);
	}

	/*
	 ** Register event sources with the queue.
	 ** Register the keyboard and the display,
	 ** as it provides the quit message from clicking
	 ** on the X in the top corner.
	 */
	al_register_event_source(event_queue, al_get_keyboard_event_source());
	al_register_event_source(event_queue, al_get_display_event_source(display));

	// create a font for writing
	ALLEGRO_FONT * font = al_create_builtin_font();
	// keyboard state for polling keys
	ALLEGRO_KEYBOARD_STATE keyboard_state;
	// an event for the event loop
	ALLEGRO_EVENT event;
	ros::NodeHandle node;
	// register a publisher for the iRat’s command velocity
	ros::Publisher pub_cmdvel =
		node.advertise<irat_msgs::IRatVelocity>("/irat_red/serial/cmdvel", 1);
	// message to publish
	irat_msgs::IRatVelocity cmdvel_msg;
	// the velocities to send to the iRat
	double vtrans = 0, vrot = 0;
	// create a rate limiter for 30Hz.
	ros::Rate rate(30.0);
	// start logging the keys pressed
	// first append the current time (in seconds)
	// to "key_log_" to create a filename with the
	// program start time.
	std::ostringstream os;
	os << "key_log_" << ros::Time::now().toSec() << ".txt";
	// open a file and write CSV headers
	// note that the path of the log file depends on what
	// directory this application is run from
	std::ofstream key_log_file(os.str().c_str());
	log_add_header(key_log_file);
	
        while (node.ok())
	{
		ros::spinOnce();
		al_clear_to_color(al_map_rgb(0, 0, 0));

		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 20, ALLEGRO_ALIGN_LEFT, "Keyboard Controller");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 30, ALLEGRO_ALIGN_LEFT, "-------------------");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 40, ALLEGRO_ALIGN_LEFT, "[Up arrow key]-forwards");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 60, ALLEGRO_ALIGN_LEFT, "[Left arrow key]-left");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 50, ALLEGRO_ALIGN_LEFT, "[Down arrow key]-backwards");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 70, ALLEGRO_ALIGN_LEFT, "[Right arrow key]-right");
		al_draw_textf(font, al_map_rgb(255, 0, 0), 20, 80, ALLEGRO_ALIGN_LEFT, "[ESC]-exit");

		// present the backbuffer
		al_flip_display();
		// check for a new event in the event queue
		if (al_get_next_event(event_queue, &event))
		{
			switch(event.type)
			{
				case ALLEGRO_EVENT_KEY_DOWN:
				case ALLEGRO_EVENT_KEY_UP:
					if (event.keyboard.keycode == ALLEGRO_KEY_ESCAPE)
					{
						// escape key pressed, so quit
						goto finish;
					}
					/*
					 ** log the key stroke if it is applicable.
					 */
					log_key_stroke(key_log_file, event);
					break;
					// clicking on the ’X’ in the top corner
				case ALLEGRO_EVENT_DISPLAY_CLOSE:
					// quit
					goto finish;
			}
		}
		// we get a copy of the keyboard state as it
		// was at this point in time
		al_get_keyboard_state(&keyboard_state);
		/*
		 ** Check to see which keys are down and set
		 ** vtrans and vrot appropriately.
		 */
		if (al_key_down(&keyboard_state, ALLEGRO_KEY_UP))
		{
			vtrans = VFORWARD;
		}
		else if (al_key_down(&keyboard_state, ALLEGRO_KEY_DOWN))
		{
			vtrans = VBACKWARD;
		}
		else
		{
			// setting the velocity for no key gives
			// a smooth falloff
			vtrans = 0.3 * vtrans;
		}
		if (al_key_down(&keyboard_state, ALLEGRO_KEY_LEFT))
		{
			vrot = 0.3;
		}
		else if (al_key_down(&keyboard_state, ALLEGRO_KEY_RIGHT))
		{
			vrot = -0.3;
		}
		else
		{
			// use a smooth falloff again
			vrot = 0.3 * vrot;
		}

		/*
		 ** Set all of the fields in the
		 ** velocity message and then publish it
		 ** to the iRat.
		 */
		cmdvel_msg.header.stamp = ros::Time::now();
		cmdvel_msg.magnitude = vtrans;
		cmdvel_msg.angle = vrot;
		pub_cmdvel.publish(cmdvel_msg);
		cmdvel_msg.header.seq++;
		rate.sleep();
	}
finish:
	return 0;
}

