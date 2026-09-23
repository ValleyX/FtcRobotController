{
  "version": 1,
  "startPoint": {
    "x": 56,
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
        "x": 60,
        "y": 14.9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 295
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
        "x": 60,
        "y": 35,
        "heading": "linear",
        "reverse": false,
        "startDeg": 295,
        "endDeg": 180
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
        "x": 23.2,
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
        "x": 60,
        "y": 14.9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 295
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
        "x": 60,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "degrees": 0,
        "startDeg": 295,
        "endDeg": 180
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
        "x": 23.8,
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
      "locked": false
    },
    {
      "id": "mklzzjtr-4k1pe1",
      "name": "INTAKE2_TO_SHOOT",
      "endPoint": {
        "x": 60,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 318
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
        "x": 42.3,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 318,
        "endDeg": 180
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
        "x": 19,
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
      "id": "mkm035ei-nk61m3",
      "name": "INTAKE3_TO_SHOOT",
      "endPoint": {
        "x": 19,
        "y": 70,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 270
      },
      "controlPoints": [],
      "color": "#9BC68D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "settings": {
    "xVelocity": 30,
    "yVelocity": 30,
    "aVelocity": 1.5707963267948966,
    "kFriction": 0.4,
    "rLength": 18,
    "rWidth": 16,
    "safetyMargin": 0,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "maxAngularAcceleration": 0,
    "fieldMap": "decode.webp",
    "fieldRotation": 0,
    "robotImage": "/robot.png",
    "javaPackageName": "org.firstinspires.ftc.teamcode.Commands.AutoCommands",
    "theme": "auto",
    "autosaveMode": "never",
    "autosaveInterval": 5,
    "showVelocityHeatmap": false,
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionSkinCurrentPathOnly": false,
    "onionLayerSpacing": 6,
    "optimizationIterations": 300,
    "optimizationPopulationSize": 100,
    "optimizationMutationRate": 0.4,
    "optimizationMutationStrength": 6,
    "validateFieldBoundaries": true,
    "restrictDraggingToField": true,
    "keyBindings": [
      {
        "id": "save-project",
        "key": "cmd+s, ctrl+s",
        "description": "Save project",
        "action": "saveProject",
        "category": "File"
      },
      {
        "id": "save-file-as",
        "key": "cmd+shift+s, ctrl+shift+s",
        "description": "Save As (download trajectory)",
        "action": "saveFileAs",
        "category": "File"
      },
      {
        "id": "export-gif",
        "key": "cmd+shift+e, ctrl+shift+e",
        "description": "Export Animated Image",
        "action": "exportGif",
        "category": "File"
      },
      {
        "id": "add-path",
        "key": "p",
        "description": "Add new path",
        "action": "addNewLine",
        "category": "Editing"
      },
      {
        "id": "add-wait",
        "key": "w",
        "description": "Add wait",
        "action": "addWait",
        "category": "Editing"
      },
      {
        "id": "add-rotate",
        "key": "t",
        "description": "Add rotate event",
        "action": "addRotate",
        "category": "Editing"
      },
      {
        "id": "add-event-marker",
        "key": "e",
        "description": "Add event marker",
        "action": "addEventMarker",
        "category": "Editing"
      },
      {
        "id": "add-control-point",
        "key": "a",
        "description": "Add control point",
        "action": "addControlPoint",
        "category": "Editing"
      },
      {
        "id": "remove-control-point",
        "key": "d",
        "description": "Remove control point",
        "action": "removeControlPoint",
        "category": "Editing"
      },
      {
        "id": "duplicate",
        "key": "shift+d, cmd+d",
        "description": "Duplicate selected item",
        "action": "duplicate",
        "category": "Editing"
      },
      {
        "id": "remove-selected",
        "key": "backspace, delete",
        "description": "Remove selected point or wait",
        "action": "removeSelected",
        "category": "Editing"
      },
      {
        "id": "undo",
        "key": "cmd+z, ctrl+z",
        "description": "Undo",
        "action": "undo",
        "category": "Editing"
      },
      {
        "id": "redo",
        "key": "cmd+shift+z, ctrl+shift+z, ctrl+y",
        "description": "Redo",
        "action": "redo",
        "category": "Editing"
      },
      {
        "id": "move-point-up",
        "key": "i, up",
        "description": "Move selected point up",
        "action": "movePointUp",
        "category": "Editing"
      },
      {
        "id": "move-point-down",
        "key": "k, down",
        "description": "Move selected point down",
        "action": "movePointDown",
        "category": "Editing"
      },
      {
        "id": "move-point-left",
        "key": "j, left",
        "description": "Move selected point left",
        "action": "movePointLeft",
        "category": "Editing"
      },
      {
        "id": "move-point-right",
        "key": "l, right",
        "description": "Move selected point right",
        "action": "movePointRight",
        "category": "Editing"
      },
      {
        "id": "increase-val",
        "key": "=",
        "description": "Increase value (wait/rotate)",
        "action": "increaseValue",
        "category": "Editing"
      },
      {
        "id": "decrease-val",
        "key": "-",
        "description": "Decrease value (wait/rotate)",
        "action": "decreaseValue",
        "category": "Editing"
      },
      {
        "id": "add-obstacle",
        "key": "shift+o",
        "description": "Add new obstacle",
        "action": "addObstacle",
        "category": "Editing"
      },
      {
        "id": "focus-name",
        "key": "f2, enter",
        "description": "Rename selected item",
        "action": "focusName",
        "category": "Editing"
      },
      {
        "id": "deselect-all",
        "key": "escape",
        "description": "Deselect all",
        "action": "deselectAll",
        "category": "Navigation"
      },
      {
        "id": "toggle-heading-mode",
        "key": "shift+h",
        "description": "Toggle Heading Mode",
        "action": "toggleHeadingMode",
        "category": "Editing"
      },
      {
        "id": "toggle-reverse",
        "key": "shift+r",
        "description": "Toggle Reverse",
        "action": "toggleReverse",
        "category": "Editing"
      },
      {
        "id": "toggle-lock",
        "key": "shift+l",
        "description": "Toggle Locked State",
        "action": "toggleLock",
        "category": "Editing"
      },
      {
        "id": "play-pause",
        "key": "space",
        "description": "Play / Pause",
        "action": "togglePlay",
        "category": "Playback"
      },
      {
        "id": "increase-speed",
        "key": "shift+up",
        "description": "Increase playback speed",
        "action": "increasePlaybackSpeed",
        "category": "Playback"
      },
      {
        "id": "decrease-speed",
        "key": "shift+down",
        "description": "Decrease playback speed",
        "action": "decreasePlaybackSpeed",
        "category": "Playback"
      },
      {
        "id": "reset-playback-speed",
        "key": "1",
        "description": "Reset playback speed",
        "action": "resetPlaybackSpeed",
        "category": "Playback"
      },
      {
        "id": "step-back",
        "key": "shift+left",
        "description": "Step animation backward",
        "action": "stepBackward",
        "category": "Playback"
      },
      {
        "id": "step-forward",
        "key": "shift+right",
        "description": "Step animation forward",
        "action": "stepForward",
        "category": "Playback"
      },
      {
        "id": "reset-animation",
        "key": "r",
        "description": "Reset animation",
        "action": "resetAnimation",
        "category": "Playback"
      },
      {
        "id": "toggle-onion",
        "key": "o",
        "description": "Toggle onion layers",
        "action": "toggleOnion",
        "category": "View"
      },
      {
        "id": "toggle-grid",
        "key": "g",
        "description": "Toggle grid",
        "action": "toggleGrid",
        "category": "View"
      },
      {
        "id": "cycle-grid-size",
        "key": "]",
        "description": "Cycle grid spacing",
        "action": "cycleGridSize",
        "category": "View"
      },
      {
        "id": "cycle-grid-size-prev",
        "key": "[",
        "description": "Cycle grid spacing backward",
        "action": "cycleGridSizeReverse",
        "category": "View"
      },
      {
        "id": "toggle-snap",
        "key": "n",
        "description": "Toggle snap to grid",
        "action": "toggleSnap",
        "category": "View"
      },
      {
        "id": "toggle-protractor",
        "key": "shift+p",
        "description": "Toggle protractor",
        "action": "toggleProtractor",
        "category": "View"
      },
      {
        "id": "zoom-in",
        "key": "cmd+=, ctrl+=, cmd+shift+=, ctrl+shift+=, cmd+num_add, ctrl+num_add",
        "description": "Zoom In",
        "action": "zoomIn",
        "category": "View"
      },
      {
        "id": "zoom-out",
        "key": "cmd+-, ctrl+-, cmd+num_subtract, ctrl+num_subtract",
        "description": "Zoom Out",
        "action": "zoomOut",
        "category": "View"
      },
      {
        "id": "zoom-reset",
        "key": "cmd+0, ctrl+0, cmd+num_0, ctrl+num_0",
        "description": "Reset Zoom",
        "action": "zoomReset",
        "category": "View"
      },
      {
        "id": "toggle-collapse-all",
        "key": "shift+c",
        "description": "Toggle collapse/expand all",
        "action": "toggleCollapseAll",
        "category": "View"
      },
      {
        "id": "toggle-sidebar",
        "key": "b",
        "description": "Toggle sidebar / control tab",
        "action": "toggleSidebar",
        "category": "View"
      },
      {
        "id": "toggle-velocity-heatmap",
        "key": "v",
        "description": "Toggle Velocity Heatmap",
        "action": "toggleVelocityHeatmap",
        "category": "View"
      },
      {
        "id": "select-next",
        "key": "tab",
        "description": "Select next item",
        "action": "selectNext",
        "category": "Navigation"
      },
      {
        "id": "select-prev",
        "key": "shift+tab",
        "description": "Select previous item",
        "action": "selectPrev",
        "category": "Navigation"
      },
      {
        "id": "select-paths-tab",
        "key": "alt+1",
        "description": "Switch to Paths tab",
        "action": "selectTabPaths",
        "category": "Navigation"
      },
      {
        "id": "select-field-tab",
        "key": "alt+2",
        "description": "Switch to Field & Tools tab",
        "action": "selectTabField",
        "category": "Navigation"
      },
      {
        "id": "select-table-tab",
        "key": "alt+3",
        "description": "Switch to Table tab",
        "action": "selectTabTable",
        "category": "Navigation"
      },
      {
        "id": "cycle-tabs-next",
        "key": "ctrl+tab",
        "description": "Cycle tabs forward",
        "action": "cycleTabNext",
        "category": "Navigation"
      },
      {
        "id": "cycle-tabs-prev",
        "key": "ctrl+shift+tab",
        "description": "Cycle tabs backward",
        "action": "cycleTabPrev",
        "category": "Navigation"
      },
      {
        "id": "optimize-start",
        "key": "ctrl+shift+o",
        "description": "Start optimization",
        "action": "optimizeStart",
        "category": "Optimization"
      },
      {
        "id": "optimize-stop",
        "key": "ctrl+.",
        "description": "Stop optimization",
        "action": "optimizeStop",
        "category": "Optimization"
      },
      {
        "id": "optimize-apply",
        "key": "ctrl+enter",
        "description": "Apply optimized path",
        "action": "optimizeApply",
        "category": "Optimization"
      },
      {
        "id": "optimize-discard",
        "key": "ctrl+backspace",
        "description": "Discard optimization",
        "action": "optimizeDiscard",
        "category": "Optimization"
      },
      {
        "id": "optimize-retry",
        "key": "ctrl+shift+r",
        "description": "Retry optimization",
        "action": "optimizeRetry",
        "category": "Optimization"
      },
      {
        "id": "open-settings",
        "key": "cmd+, ctrl+,",
        "description": "Open Settings",
        "action": "openSettings",
        "category": "Settings"
      },
      {
        "id": "show-help",
        "key": "shift+/",
        "description": "Show keyboard shortcuts",
        "action": "showHelp",
        "category": "Settings"
      },
      {
        "id": "toggle-stats",
        "key": "s",
        "description": "Toggle Path Statistics",
        "action": "toggleStats",
        "category": "View"
      },
      {
        "id": "focus-x",
        "key": "x",
        "description": "Focus X Input",
        "action": "focusX",
        "category": "Editing"
      },
      {
        "id": "focus-y",
        "key": "y",
        "description": "Focus Y Input",
        "action": "focusY",
        "category": "Editing"
      },
      {
        "id": "focus-heading",
        "key": "h",
        "description": "Focus Heading Input",
        "action": "focusHeading",
        "category": "Editing"
      },
      {
        "id": "open-whats-new",
        "key": "shift+n",
        "description": "Open What's New",
        "action": "openWhatsNew",
        "category": "Settings"
      },
      {
        "id": "toggle-command-palette",
        "key": "cmd+p, ctrl+p",
        "description": "Toggle Command Palette",
        "action": "toggleCommandPalette",
        "category": "View"
      },
      {
        "id": "open-file",
        "key": "cmd+o, ctrl+o",
        "description": "Open File",
        "action": "openFile",
        "category": "File"
      },
      {
        "id": "new-file",
        "key": "cmd+n, ctrl+n, cmd+r, ctrl+r",
        "description": "New Project / Reset Path",
        "action": "newProject",
        "category": "File"
      },
      {
        "id": "toggle-file-manager",
        "key": "cmd+shift+m, ctrl+shift+m",
        "description": "Toggle File Manager",
        "action": "toggleFileManager",
        "category": "File"
      },
      {
        "id": "export-java",
        "key": "cmd+shift+j, ctrl+shift+j",
        "description": "Export Java Code",
        "action": "exportJava",
        "category": "Export"
      },
      {
        "id": "export-points",
        "key": "cmd+shift+k, ctrl+shift+k",
        "description": "Export Points Array",
        "action": "exportPoints",
        "category": "Export"
      },
      {
        "id": "export-sequential",
        "key": "cmd+shift+u, ctrl+shift+u",
        "description": "Export Sequential Command",
        "action": "exportSequential",
        "category": "Export"
      },
      {
        "id": "export-pp",
        "key": "cmd+shift+x, ctrl+shift+x",
        "description": "Export .pp (JSON)",
        "action": "exportPP",
        "category": "Export"
      },
      {
        "id": "set-file-manager-directory",
        "key": "",
        "description": "Set File Manager Directory",
        "action": "setFileManagerDirectory",
        "category": "File"
      },
      {
        "id": "reset-keybinds",
        "key": "",
        "description": "Reset Keybinds",
        "action": "resetKeybinds",
        "category": "Settings"
      },
      {
        "id": "reset-settings",
        "key": "",
        "description": "Reset Settings",
        "action": "resetSettings",
        "category": "Settings"
      },
      {
        "id": "cycle-theme",
        "key": "shift+t",
        "description": "Cycle Light/Dark Mode",
        "action": "cycleTheme",
        "category": "View"
      },
      {
        "id": "show-debug",
        "key": "",
        "description": "Show/Hide Debug Sequence",
        "action": "toggleDebugSequence",
        "category": "View"
      },
      {
        "id": "toggle-bounds",
        "key": "",
        "description": "Toggle Field Boundaries",
        "action": "toggleFieldBoundaries",
        "category": "View"
      },
      {
        "id": "toggle-drag",
        "key": "",
        "description": "Toggle Drag Restriction",
        "action": "toggleDragRestriction",
        "category": "View"
      },
      {
        "id": "pan-start",
        "key": "home",
        "description": "Pan to Start Point",
        "action": "panToStart",
        "category": "View"
      },
      {
        "id": "pan-end",
        "key": "end",
        "description": "Pan to End Point",
        "action": "panToEnd",
        "category": "View"
      },
      {
        "id": "select-last",
        "key": "",
        "description": "Select Last Point",
        "action": "selectLast",
        "category": "Editing"
      },
      {
        "id": "reset-start",
        "key": "",
        "description": "Reset Start Point",
        "action": "resetStartPoint",
        "category": "Editing"
      },
      {
        "id": "snap-select",
        "key": "shift+s",
        "description": "Snap Selection to Grid",
        "action": "snapSelection",
        "category": "Editing"
      },
      {
        "id": "copy-json",
        "key": "",
        "description": "Copy Path JSON to Clipboard",
        "action": "copyPathJson",
        "category": "Export"
      },
      {
        "id": "theme-light",
        "key": "",
        "description": "Set Theme: Light",
        "action": "setThemeLight",
        "category": "Settings"
      },
      {
        "id": "theme-dark",
        "key": "",
        "description": "Set Theme: Dark",
        "action": "setThemeDark",
        "category": "Settings"
      },
      {
        "id": "auto-save-never",
        "key": "",
        "description": "Autosave: Never",
        "action": "setAutoSaveNever",
        "category": "Settings"
      },
      {
        "id": "auto-save-1m",
        "key": "",
        "description": "Autosave: 1 Minute",
        "action": "setAutoSave1m",
        "category": "Settings"
      },
      {
        "id": "auto-save-5m",
        "key": "",
        "description": "Autosave: 5 Minutes",
        "action": "setAutoSave5m",
        "category": "Settings"
      },
      {
        "id": "auto-save-change",
        "key": "",
        "description": "Autosave: On Change",
        "action": "setAutoSaveChange",
        "category": "Settings"
      },
      {
        "id": "auto-save-close",
        "key": "",
        "description": "Autosave: On Close",
        "action": "setAutoSaveClose",
        "category": "Settings"
      },
      {
        "id": "docs",
        "key": "",
        "description": "Open Documentation",
        "action": "openDocs",
        "category": "Help"
      },
      {
        "id": "issues",
        "key": "",
        "description": "Report Issue",
        "action": "reportIssue",
        "category": "Help"
      }
    ],
    "recentFiles": [
      "C:\\Users\\ftcte\\StudioProjects\\FtcRobotController-new\\Team12841\\src\\main\\assets\\AutoPaths\\BlueFar.pp",
      "C:\\Users\\ftcte\\StudioProjects\\FtcRobotController-new\\Team12841\\src\\main\\assets\\AutoPaths\\RedClose.pp",
      "C:\\Users\\ftcte\\StudioProjects\\FtcRobotController-new\\Team12841\\src\\main\\assets\\AutoPaths\\BlueClose.pp"
    ],
    "lastSeenVersion": "1.6.2",
    "showDebugSequence": false,
    "fileManagerSortMode": "date"
  },
  "sequence": [
    {
      "kind": "path",
      "lineId": "mkk6np3a-6t2w5x"
    },
    {
      "kind": "wait",
      "id": "mklzsnrx-nnkqhk",
      "name": "SHOOT (1)",
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
      "name": "SHOOT (2)",
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
      "name": "SHOOT (3)",
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
      "lineId": "mkm035ei-nk61m3"
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
  ]
}