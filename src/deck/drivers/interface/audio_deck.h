#ifndef _AUDIO_DECK_H_
#define _AUDIO_DECK_H_

#include "deck_core.h"

void audio_deckInit(DeckInfo* info);

bool audio_deckTest(void);
void audio_deckTask(void* arg);

/* This type MUST be 8-bit */
typedef unsigned char	BYTE;

#endif /* _AUDIO_DECK_H_ */
