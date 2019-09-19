/*
 * external_led.c
 *
 *  Created on: Jul 25, 2017
 *      Author: Manpuria
 */

#include "deck.h"


void external_led_init(DeckInfo *info)
{
	pinMode(DECK_GPIO_IO4,OUTPUT);
	digitalWrite(DECK_GPIO_IO4, HIGH);

}

bool test_external_led()
{
	return 1;
}

const DeckDriver external_led_driver ={
		.vid = 0,
		.pid = 0,
		.name = "deck_external_led",

		.usedGpio = DECK_USING_IO_4,

		.init = external_led_init,
		.test = test_external_led,

};

DECK_DRIVER(external_led_driver);
