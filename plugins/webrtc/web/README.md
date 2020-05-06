## Overview

This directory contains a simple javascript WebRTC client with the
accompanying python3 web server.

## Usage

Copy the certificates to be used by the server in this directory or generate 
them here with the following command.

```sh
../util/generate_cert.sh
```


Start the web server with

```sh
./server.py # or python server.py
```

Visit the webpage at `https://localhost:8000`. The webpage is also available
on the local network.

## Possible issues

**Permissions**

The user accessing the website must have the permissions to access the camera 
and the microphone on the computer they are using.

**Certificate**

You will be prompted to access the certificate.

**Security**

If the website is accessed with secure http, the client won't be able to access 
the camera or the microphone.
