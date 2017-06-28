# OSVR-Opentrack
Opentrack plugin for OSVR

# How to use
1. Add the following to your OSVR Server config
```
"drivers": [{
		"plugin": "inf_osvr_opentrack",
		"driver": "OpenTracker"
	}
	...
	],
	"aliases": {
		"/me/head": "/inf_osvr_opentrack/OpenTracker/semantic/tracker"
	}
```

2. Open Opentrack and set "Output" to "UDP over network" and in the settings window, set the IP to 127.0.0.1 (or the ip of the pc that has the osvr server running) and port to 4242.
3. Click start in Opentrack, and open the OSVR Server.