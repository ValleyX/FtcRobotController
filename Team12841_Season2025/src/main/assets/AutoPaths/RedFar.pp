{
  "startPoint": {
    "x": 79,
    "y": 9,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "mkk6np3a-6t2w5x",
      "name": "START_TO_PRELOAD",
      "endPoint": {
        "x": 84,
        "y": 14.9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 247
      },
      "controlPoints": [],
      "color": "#C9D7AD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mklzt20v-rdu3fw",
      "name": "INTAKE1_ALIGN",
      "endPoint": {
        "x": 84,
        "y": 35,
        "heading": "linear",
        "reverse": false,
        "startDeg": 247,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#CDBB65",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "locked": false
    },
    {
      "id": "mklzun1x-a6u6i3",
      "name": "INTAKE1",
      "endPoint": {
        "x": 120.8,
        "y": 35,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#5DDADA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "locked": true
    },
    {
      "id": "mklzvsa1-6qsgd8",
      "name": "INTAKE1_TO_SHOOT",
      "endPoint": {
        "x": 84,
        "y": 14.9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 247
      },
      "controlPoints": [],
      "color": "#6C8598",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mklzxg95-twzwmx",
      "name": "INTAKE2_ALIGN",
      "endPoint": {
        "x": 84,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "degrees": 0,
        "startDeg": 247,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#ACBB7A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mklzyx9r-o8t4ey",
      "name": "INTAKE2",
      "endPoint": {
        "x": 120.8,
        "y": 60,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#9C68A9",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "locked": true
    },
    {
      "id": "mklzzjtr-4k1pe1",
      "name": "INTAKE2_TO_SHOOT",
      "endPoint": {
        "x": 84,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 225
      },
      "controlPoints": [],
      "color": "#869585",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkm060km-k6wkkt",
      "name": "INTAKE3_ALIGN",
      "endPoint": {
        "x": 101.7,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 225,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#7B8DAC",
      "eventMarkers": [],
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkm01d9d-um5c9j",
      "name": "INTAKE3",
      "endPoint": {
        "x": 125,
        "y": 84,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#99AA7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "locked": true
    },
    {
      "id": "mkm10hto-24bu2l",
      "endPoint": {
        "x": 125,
        "y": 70,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 270
      },
      "controlPoints": [],
      "color": "#798BDC",
      "name": "MOVE_AWAY",
      "eventMarkers": [],
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
      "lineId": "mkk6np3a-6t2w5x"
    },
    {
      "kind": "wait",
      "id": "mklzsnrx-nnkqhk",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false,
      "_linkedName": "SHOOT"
    },
    {
      "kind": "path",
      "lineId": "mklzt20v-rdu3fw"
    },
    {
      "kind": "path",
      "lineId": "mklzun1x-a6u6i3"
    },
    {
      "kind": "path",
      "lineId": "mklzvsa1-6qsgd8"
    },
    {
      "kind": "wait",
      "id": "mklzwwtz-9tnmo8",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false,
      "_linkedName": "SHOOT"
    },
    {
      "kind": "path",
      "lineId": "mklzxg95-twzwmx"
    },
    {
      "kind": "path",
      "lineId": "mklzyx9r-o8t4ey"
    },
    {
      "kind": "path",
      "lineId": "mklzzjtr-4k1pe1"
    },
    {
      "kind": "wait",
      "id": "mkm00ys0-tj7ngl",
      "name": "SHOOT",
      "durationMs": 3000,
      "locked": false,
      "_linkedName": "SHOOT"
    },
    {
      "kind": "path",
      "lineId": "mkm060km-k6wkkt"
    },
    {
      "kind": "path",
      "lineId": "mkm01d9d-um5c9j"
    },
    {
      "kind": "path",
      "lineId": "mkm10hto-24bu2l"
    }
  ]
}