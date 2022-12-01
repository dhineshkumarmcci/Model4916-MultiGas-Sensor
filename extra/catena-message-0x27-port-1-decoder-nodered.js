/*
Name:   model4916-decoder-ttn.js

Function:
    This function decodes the record (port 1, format 0x27) sent by the
    MCCI Model 4916 multigas and environment sensor application.

Copyright and License:
    See accompanying LICENSE file

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022
*/
function dewpoint(t, rh) {
    var c1 = 243.04;
    var c2 = 17.625;
    var h = rh / 100;
    if (h <= 0.01)
        h = 0.01;
    else if (h > 1.0)
        h = 1.0;

    var lnh = Math.log(h);
    var tpc1 = t + c1;
    var txc2 = t * c2;
    var txc2_tpc1 = txc2 / tpc1;

    var tdew = c1 * (lnh + txc2_tpc1) / (c2 - lnh - txc2_tpc1);
    return tdew;
}

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    if (port === 1) {
        cmd = bytes[0];
        if (cmd == 0x27) {
            // i is used as the index into the message. Start with the flag byte.
            var i = 1;
            // fetch the bitmap.
            var flags = bytes[i++];

            if (flags & 0x1) {
                // set vRaw to a uint16, and increment pointer
                var vRaw = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                // interpret uint16 as an int16 instead.
                if (vRaw & 0x8000)
                    vRaw += -0x10000;
                // scale and save in decoded.
                decoded.vBat = vRaw / 4096.0;
            }

            if (flags & 0x2) {
                var iBoot = bytes[i];
                i += 1;
                decoded.boot = iBoot;
            }

            if (flags & 0x8) {
                // we have temp, pressure, RH
                var tRaw = (bytes[i] << 8) + bytes[i + 1];
                if (tRaw & 0x8000)
                    tRaw = -0x10000 + tRaw;
                i += 2;
                var pRaw = (bytes[i] << 8) + bytes[i + 1];
                i += 2;
                var hRaw = bytes[i++];

                decoded.tempC = tRaw / 256;
                decoded.error = "none";
                decoded.p = pRaw * 4 / 100.0;
                decoded.rh = hRaw / 256 * 100;
                decoded.tDewC = dewpoint(decoded.tempC, decoded.rh);
            }

			// To be continued

        } else {
            node.error("not ours! " + bytes[0].toString());
            return null;
        }
    }
    return decoded;
}

var bytes;

if ("payload_raw" in msg) {
    // the console already decoded this
    bytes = msg.payload_raw;  // pick up data for convenience
    // msg.payload_fields still has the decoded data from ttn
} else {
    // no console decode
    bytes = msg.payload;  // pick up data for conveneince
}

// try to decode.
var result = Decoder(bytes, msg.port);

if (result === null) {
    node.error("not port 1/fmt 0x27! port=" + msg.port.toString());
}

// now update msg with the new payload and new .local field
// the old msg.payload is overwritten.
msg.payload = result;
msg.local =
    {
        nodeType: "Model 4916",
        platformType: "Model 4916",
        radioType: "Murata",
        applicationName: "MultiGas sensor"
    };

return msg;