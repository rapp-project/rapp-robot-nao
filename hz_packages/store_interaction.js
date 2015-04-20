
// store_interaction.js
// Downloading applications (HOP weblets) from the cloud
// (RAPP store)
//
// Marcin Szlenk <m.szlenk@elka.pw.edu.pl>
// Copyright 2014 RAPP
//
// Run: hop store_interaction.js

var HOP = require ("hop");
var HOPHZ = require ("hophz");

// Just for 'foo.resource' property (see below).
service foo () {}

// URL of the weblet repositories on the RAPP store.
// NOTE: the below URL is for testing purposes only;
// it just points to where this source file lies,
// assuming the RAPP store (HOP server) is running
// on the local machine on port 9999.
var RAPPStoreURL = "http://RAPPSTORE_IP_ADDRESS:PORT/PATH_TO_HZ_PACKAGES";

// Download and install a hz package from a given URL,
// where:
//
// url - URL path of a hz package to be installed,
// e.g. "http://rappstore.org:8080/hello-1.0.0.hz".
//
// Returns true on success and false on failure.
function hophzDownload (url) {
  try {
    return HOPHZ.download (url, "localhost");
  } catch (e) {
    return false;
  }
}

// Download and install a given hz package (a rapp
// application) from the RAPP store
//
// hzPackageName - the name of a hz package to be
// installed from the RAPP store, e.g. "hello-1.0.0.hz".
//
// Returns the path to the successfully installed package
// or an empty string on failure.
function download (hzPackageName) {

  var hzPackageURL = RAPPStoreURL + hzPackageName;

  var result = hophzDownload( hzPackageURL );

	if (result === true) {
    // NOTE: the below path can be changed in HOP preferences,
    // so it rather shouldn't be hard-coded here.
    var webletInstallationPath = "~/.config/hop/weblets/";

    // Remove the "-x.x.x.hz" suffix.
    // NOTE: in general, the hz package names can be more complex
    // (see HOP documentation)
    var packageName =
      hzPackageName.slice (0, hzPackageName.indexOf("-"));

    return webletInstallationPath + packageName;
  }
	else {
    return "";
  }
}

// Name of the ROS topic for receiving requests.
var requestTopic  = "/rapp/store_interaction/request";
// Name of the ROS topic for publishing responses.
var responseTopic = "/rapp/store_interaction/response";

// Return an opened WebSocket to ROSbridge, with all the handlers
// installed.
function connectToRosbridge (rosbridgeURL)
{
  var ws = new WebSocket (rosbridgeURL);
  console.log ("ROSbridge connection opened");

  // Handle the 'open' event on the WebSocket.
  ws.onopen = function (event)
  {
    var msg =
    {
      "op": "subscribe",
      "topic": requestTopic,
      "type": "std_msgs/String"
    };
    ws.send (JSON.stringify (msg));

    // Advertise that we will be publishing to the response topic.
    msg =
    {
      "op": "advertise",
      "topic": responseTopic,
      "type": "std_msgs/String"
    };
    ws.send (JSON.stringify (msg));

    console.log ("Websocket opened");
  }

  // Handle WebSocket messages.
  ws.onmessage = function (event)
  {
    var text = event.value;

    // Convert JSON text into JavaScript object.
    var value = eval ("(" + text + ")");

    if (value.op === "publish")
    {
      if (value.topic === requestTopic)
      {
        var msg = value.msg;
        var hzPackageName = msg.data;

        console.log ("Downloading '%s' requested", hzPackageName);

        var result = download (hzPackageName);

        if (result === "")
          console.log ("Downloading failed");
        else
          console.log ("Package installed in: %s", result);

        // Publish response.
        msg =
        {
          "op": "publish",
          "topic": responseTopic,
          "msg": { "data": result } // ROS message.
        };
        ws.send (JSON.stringify (msg));
      }
    }
  }

  ws.onclose = function (event)
  {
    console.log ("ROSbridge connection closed");
  }

  return ws;
}

// The ROSbridge WebSocket URL.
var rosbridgeURL = "ws://localhost:9090";

// Open a client websocket to ROSbridge.
var rosbridge = connectToRosbridge (rosbridgeURL);

console.log ("Robot-store interaction started");
console.log ("ROS topic for sending requests: %s", requestTopic);
console.log ("ROS topic for receiving responses: %s", responseTopic);
