-------------------------------------------------------------------------------
Handsfree app
-------------------------------------------------------------------------------

Overview
--------
This app sample app performs as a Bluetooth Handsfree device

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board
2. Use ClientControl application to send various commands


BR/EDR
- To find BR/EDR devices: Click on "Start BR/EDR Discovery"

Handsfree Connection
- To create handsfree connection to remote Audio Gateway (AG) device (such as mobile phone),
  choose the Bluetooth address of the remote AG device from the BR/EDR combo box
- Click "Connect" button under Handsfree
- OR Put the device in discoverable and connectable mode and search for the device from AG device and connect
- The following HF operations can be performed using the client control application
     Connect / Disconnect HF or SCO connection
     Answer / Hang-up the call
     Dial / Redial the number
     Control Held call (ex. hold call, release all held calls, etc.)
     Mic / Speaker gain control
-------------------------------------------------------------------------------
