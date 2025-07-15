# Speechrecognition

## Getting Started

### 1. Microphone access

make sure that the Python Code in the Dev Container has access to the microphone of the host, to record Audio with PyAudio

Test with

```
arecord -l
```

if microphone works on hostsystem

the output should be something like:

```
**** List of CAPTURE Hardware Devices ****
card 0: PCH [HDA Intel PCH], device 0: ALC1150 Analog [ALC1150 Analog]
  Subdevices: 0/1
  Subdevice #0: subdevice #0
card 0: PCH [HDA Intel PCH], device 2: ALC1150 Alt Analog [ALC1150 Alt Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

### 2. check user groups 

```
*groups $USER*
```

the output should be something like:

```
ubuntu : ubuntu adm dialout cdrom floppy sudo audio dip video plugdev
```

if group audio is missing:

```
sudo usermod -aG audio $USER
```
### 3. check if audio device are visible

```
ls -l /dev/snd
```

the output should consist of lines of codes like:

```
total 0
crw-rw----+ 1 root audio 116, 7 Jul 14 14:00 pcmC0D0c
crw-rw----+ 1 root audio 116, 6 Jul 14 14:00 pcmC0D0p
crw-rw----+ 1 root audio 116, 5 Jul 14 14:00 controlC0
```


### 4. edit devcontainer-configuration

in the project folder you can find the file ```.devcontainer/devcontainer.json```

add the following to the runarc-block

```
"runArgs": [
  "--device=/dev/snd",
  "--group-add", "audio",
  "-e", "PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native",
  "-v", "${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native",
  "-v", "/etc/machine-id:/etc/machine-id"
]
```


after you save with Strg + S,
 reboot container with:

Strg + Shift + P and click ```Dev Container: Rebuild Container```

### 5. Test PyAudio

create a file in your container z.B. mic_test.py with the following code:

```
import pyaudio

p = pyaudio.PyAudio()

for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"{i}: {info['name']} ({info['maxInputChannels']} Kanäle)")

```

the output should consist of lines of codes like:

```
0: HDA Intel PCH: ALC256 Analog (2 Kanäle) and should contain the devices:
25: pulse (32 Kanäle)
26: a52 (0 Kanäle)
27: default (32 Kanäle)

```