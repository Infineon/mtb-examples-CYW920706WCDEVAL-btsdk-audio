-------------------------------------------------------------------------------
Headset app
-------------------------------------------------------------------------------

Overview
--------
This app demonstrates the use of Bluetooth Headset via the BT audio libraries.

The sample app performs as a Bluetooth A2DP sink and AVRCP Controller (and Target for absolute volume control).
Features demonstrated
 - WICED BT AV (A2DP/AVRCP) APIs
 - WICED BT GATT APIs (implemention of Vendor Specific Device)
 - Handling of the UART WICED protocol
 - SDP and GATT descriptor/attribute configuration

See chip specific readme for more information about the BT SDK.

Instructions
------------
This application can run as a standalone Headset or in association with ClientControl.
When running in standalone mode, it is required that 20706 controls the codec chip, Currently this is not implemented.
In association with an ClientControl (apps processor emulation), this app serves to abstract the details of Bluetooth protocols and profiles while allowing ClientControl to deal with the business logic.
ClientControl is typically connected over UART and can send commands and receive notifications.

On startup this demo:
 - Initializes the Bluetooth sub system
 - Receive NVRAM information from the host

Application Instructions
->To use this application in conjunction with ClientControl:
 - Build and download application using IDE or command line build.
 - To build a downloadable hcd file, uncomment 'export DIRECT_LOAD=1' in the application makefile.
   The HCD file thus generated is downloaded into 20706 RAM. Use Client Control for testing on PCs.
 - Ensure that STANDALONE_HEADSET_APP flag is disabled.

->To use this in stand alone mode:
 - Ensure that STANDALONE_HEADSET_APP flag is enabled.
 - Button configured to demonstrate multiple functionality.
 - Button has 2 states, short press and long press. Both of them have certain features configured.
 - Button Short Press Cases:
   Play/Pause music streaming.
   Accept/End incoming call/ongoing call.
   End an outgoing call.
 - Button Long Press Cases:
   Last number redialing when in IDLE state.
   Reject an incoming call.
   Put an existing call on hold.

Application also demonstrates Vendor Specific Device (hello sensor) that
processes write requests from the client.

-------------------------------------------------------------------------------
