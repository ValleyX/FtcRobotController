{
  "startPoint": {
    "x": 117.5,
    "y": 129.1,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-qa3jtq4ctm",
      "name": "START_TO_SHOOT",
      "endPoint": {
        "x": 84.5,
        "y": 84,
        "heading": "linear",
        "startDeg": 216,
        "endDeg": 230
      },
      "controlPoints": [],
      "color": "#ff0000",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk5x80r-9xz767",
      "name": "INTAKE1",
      "endPoint": {
        "x": 128,
        "y": 84,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#6A6876",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk605ms-7g4ik5",
      "name": "INTAKE1_TO_SHOOT",
      "endPoint": {
        "x": 84.5,
        "y": 84,
        "heading": "linear",
        "reverse": true,
        "startDeg": 0,
        "endDeg": 230
      },
      "controlPoints": [],
      "color": "#DC97A5",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk6579c-m4nfq2",
      "name": "SHOOT1_TO_INTAKE2_ALIGN",
      "endPoint": {
        "x": 101,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "startDeg": 230,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#A779CB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk66wem-9n0wp8",
      "name": "INTAKE2",
      "endPoint": {
        "x": 135,
        "y": 60,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#D8689B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk6ebpo-wffu5p",
      "name": "INTAKE2_TO_SHOOT",
      "endPoint": {
        "x": 84.5,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 230
      },
      "controlPoints": [],
      "color": "#88AC8B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk6np3a-6t2w5x",
      "name": "MOVE_AWAY",
      "endPoint": {
        "x": 84,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 230,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#C9D7AD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-qa3jtq4ctm"
    },
    {
      "kind": "wait",
      "id": "mkk5z4gj-z1347k",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkk5x80r-9xz767"
    },
    {
      "kind": "path",
      "lineId": "mkk605ms-7g4ik5"
    },
    {
      "kind": "wait",
      "id": "mkk652sv-p752md",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkk6579c-m4nfq2"
    },
    {
      "kind": "path",
      "lineId": "mkk66wem-9n0wp8"
    },
    {
      "kind": "path",
      "lineId": "mkk6ebpo-wffu5p"
    },
    {
      "kind": "wait",
      "id": "mkk6igbv-t7w5og",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkk6np3a-6t2w5x"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 18,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "dark",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-20T02:01:57.492Z"
}