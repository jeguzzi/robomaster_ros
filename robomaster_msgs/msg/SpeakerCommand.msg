# An command to stop or play a predefined sound or an audio file from the speaker.


uint8 CONTROL_STOP = 0  # Stop playing sound
uint8 CONTROL_PLAY = 1  # Start playing sound after stopping currently played sounds
uint8 CONTROL_ADD  = 2  # Start playing sound, without stopping currently played sounds

int32 sound_id 1        # ID of the sound to play
uint8 times 1           # how many times to play the same sound (0: infinite loop)
uint8 control           # Control command: one of ``CONTROL_STOP``, ``CONTROL_PLAY``, ``CONTROL_ADD``
string file ""          # if not empry, the local file to be played.
