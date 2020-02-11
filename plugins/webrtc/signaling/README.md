## Overview

This directory contains code for a simple python3 server that can be used for
signaling between WebRTC peers.

See Server.md for the functioning of the server.

See Signaling.md for information to send during signaling.

## Usage

In the ternimal, go the current directory. Then create the environment and install the dependencies

```sh
python3 -m venv venv # <-- you may need to install with "sudo apt install python3-venv"
source venv/bin/activate
pip3 install -r requirements.txt
```

Copy the certificates to be used by the server in this directory or
generate them by running the following command in this directory.

```sh
../util/generate_cert.sh
```

Run the server

```sh
./simple-server.py
```

or

```sh
python3 simple-server.py
```

## Possible issues

**Certificate**
- When the server is using a certificate generated locally,
  some web browsers might be unable to access the websocket server until the
  certificate has been reviewed and accepted. To do so, using the above
  example, open `https://localhost:8443` in the browser and accept the certificate.
  Update the address and port number according to the arguments passed to the 
  server at launch.
