from vosk import Model, KaldiRecognizer
import pyaudio
import pyttsx3
import json
from Pluto import pluto

model = Model(r"path/to/model")
recognizer = KaldiRecognizer(model, 16000)

mic = pyaudio.PyAudio()
stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
stream.start_stream()

my_pluto = pluto()

engine = pyttsx3.init()
engine.setProperty('rate', 150)  # Speed of speech
engine.setProperty('volume', 0.9)  # Volume level (0.0 to 1.0)

while True:
    data = stream.read(8192)
    

    if recognizer.AcceptWaveform(data):
        # print("Reading", data)
        text = recognizer.Result()
        data = json.loads(text)
        text = data["text"]
        print(text)
        if text == "":
            engine.say("Could not understand, please speak again!")
            engine.runAndWait()            
        else:
            if text == "hello":
                my_pluto.arm()
                my_pluto.take_off()
            engine.say("You said " + text)
            engine.runAndWait()