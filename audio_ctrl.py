import pygame
import os
import random
import shutil
import subprocess
import threading
import time
import yaml

try:
	import pyttsx3
	_HAS_PYTTSX3 = True
except ImportError:
	pyttsx3 = None
	_HAS_PYTTSX3 = False

usb_connected = False

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
	config = yaml.safe_load(yaml_file)

current_path = os.path.abspath(os.path.dirname(__file__))

try:
	pygame.mixer.init()
	pygame.mixer.music.set_volume(config['audio_config']['default_volume'])
	usb_connected = True
	print('audio usb connected')
except Exception:
	usb_connected = False
	print('audio usb not connected')

play_audio_event = threading.Event()
min_time_bewteen_play = config['audio_config']['min_time_bewteen_play']
_speech_lock = threading.Lock()

# Optional global pyttsx3 engine (lazy). Prefer per-call engines for thread safety.
engine = None
if _HAS_PYTTSX3:
	try:
		engine = pyttsx3.init()
		engine.setProperty('rate', config['audio_config']['speed_rate'])
	except Exception as e:
		print(f'pyttsx3 init failed: {e}')
		engine = None


def play_audio(input_audio_file):
	if not usb_connected:
		return
	try:
		pygame.mixer.music.load(input_audio_file)
		pygame.mixer.music.play()
	except Exception:
		play_audio_event.clear()
		return
	while pygame.mixer.music.get_busy():
		pass
	time.sleep(min_time_bewteen_play)
	play_audio_event.clear()


def play_random_audio(input_dirname, force_flag):
	if not usb_connected:
		return
	if play_audio_event.is_set() and not force_flag:
		return
	audio_files = [f for f in os.listdir(current_path + "/sounds/" + input_dirname) if f.endswith((".mp3", ".wav"))]
	if not audio_files:
		return
	audio_file = random.choice(audio_files)
	play_audio_event.set()
	audio_thread = threading.Thread(target=play_audio, args=(current_path + "/sounds/" + input_dirname + "/" + audio_file,))
	audio_thread.start()


def play_audio_thread(input_file):
	if not usb_connected:
		return
	if play_audio_event.is_set():
		return
	play_audio_event.set()
	audio_thread = threading.Thread(target=play_audio, args=(input_file,))
	audio_thread.start()


def play_file(audio_file):
	if not usb_connected:
		return
	audio_file = current_path + "/sounds/" + audio_file
	play_audio_thread(audio_file)


def get_mixer_status():
	if not usb_connected:
		return
	return pygame.mixer.music.get_busy()


def set_audio_volume(input_volume):
	if not usb_connected:
		return
	input_volume = float(input_volume)
	if input_volume > 1:
		input_volume = 1
	elif input_volume < 0:
		input_volume = 0
	pygame.mixer.music.set_volume(input_volume)


def set_min_time_between(input_time):
	if not usb_connected:
		return
	global min_time_bewteen_play
	min_time_bewteen_play = input_time


def _speak_espeak(text, timeout_s=25):
	"""CLI espeak — reliable on this Pi; does not require pygame USB."""
	bin_path = shutil.which('espeak') or shutil.which('espeak-ng')
	if not bin_path:
		return {'ok': False, 'backend': 'espeak', 'error': 'espeak not installed'}
	try:
		rate = int(config.get('audio_config', {}).get('speed_rate', 180) or 180)
		# espeak -s is words-per-minute-ish; clamp to a sensible range
		wpm = max(80, min(300, rate))
		proc = subprocess.run(
			[bin_path, '-v', 'en', '-s', str(wpm), str(text)],
			capture_output=True,
			text=True,
			timeout=timeout_s,
			check=False,
		)
		if proc.returncode != 0:
			err = (proc.stderr or proc.stdout or f'exit {proc.returncode}')[:200]
			return {'ok': False, 'backend': 'espeak', 'error': err}
		return {'ok': True, 'backend': 'espeak', 'error': None}
	except subprocess.TimeoutExpired:
		return {'ok': False, 'backend': 'espeak', 'error': 'espeak timeout'}
	except Exception as e:
		return {'ok': False, 'backend': 'espeak', 'error': str(e)[:200]}


def _speak_pyttsx3(text):
	"""Fresh engine per call — shared engines are not thread-safe."""
	if not _HAS_PYTTSX3:
		return {'ok': False, 'backend': 'pyttsx3', 'error': 'pyttsx3 not installed'}
	local = None
	try:
		local = pyttsx3.init()
		local.setProperty('rate', config['audio_config']['speed_rate'])
		local.say(text)
		local.runAndWait()
		return {'ok': True, 'backend': 'pyttsx3', 'error': None}
	except Exception as e:
		return {'ok': False, 'backend': 'pyttsx3', 'error': str(e)[:200]}
	finally:
		try:
			if local is not None:
				local.stop()
		except Exception:
			pass


def speak(input_text, *, force=True, block=True):
	"""Speak text. Prefer espeak CLI, then pyttsx3.

	Returns dict: {ok, backend, error, text}.
	Does **not** require pygame mixer / USB speaker for TTS.
	"""
	text = (input_text or '').strip()
	if not text:
		return {'ok': False, 'backend': None, 'error': 'empty text', 'text': ''}
	if len(text) > 400:
		text = text[:400]

	def _run():
		# Serialize TTS so concurrent say/found don't collide
		with _speech_lock:
			play_audio_event.set()
			try:
				# espeak first — works even when pygame path is HDMI-only / busy
				res = _speak_espeak(text)
				if res.get('ok'):
					return res
				res2 = _speak_pyttsx3(text)
				if res2.get('ok'):
					return res2
				# Prefer the more specific error
				err = res2.get('error') or res.get('error') or 'all TTS backends failed'
				return {
					'ok': False,
					'backend': res2.get('backend') or res.get('backend'),
					'error': err,
				}
			finally:
				play_audio_event.clear()

	if block:
		out = _run()
		out['text'] = text
		return out

	holder = {'result': None}

	def _thread_main():
		holder['result'] = _run()
		holder['result']['text'] = text

	t = threading.Thread(target=_thread_main, name='tts-speak', daemon=True)
	t.start()
	return {'ok': True, 'backend': 'async', 'error': None, 'text': text, 'thread': t}


def play_speech(input_text):
	"""Blocking TTS (legacy). Returns speak() result dict."""
	return speak(input_text, force=True, block=True)


def play_speech_thread(input_text, *, force=True):
	"""Non-blocking TTS. Always attempts speech (force default True).

	Previously silent-returned when pygame event was busy or mixer init
	failed — that dropped Seek on-found announcements. Now uses speak().
	"""
	return speak(input_text, force=force, block=False)


def stop():
	if usb_connected:
		try:
			pygame.mixer.music.stop()
		except Exception:
			pass
	play_audio_event.clear()


if __name__ == '__main__':
	print(speak('I have found the dog.'))
