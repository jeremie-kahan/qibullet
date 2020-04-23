import pyttsx

def TTS(speech):
    engine = pyttsx.init()
    engine.say(speech)
    engine.runAndWait()

#Text to be spoken
speech = "Hello, my name is Pepper. I have been designed to serve humans."
TTS(speech)
