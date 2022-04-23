"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.selectROSApi = exports.rosApi = void 0;
const unknownROS = require("./common/unknown-ros");
const ros1 = require("./ros1/ros1");
const ros2 = require("./ros2/ros2");
const ros1Api = new ros1.ROS1();
const ros2Api = new ros2.ROS2();
const unknownRosApi = new unknownROS.UnknownROS();
exports.rosApi = unknownRosApi;
function selectROSApi(version) {
    exports.rosApi = unknownRosApi;
    switch (version.trim()) {
        case "1": {
            exports.rosApi = ros1Api;
            break;
        }
        case "2": {
            exports.rosApi = ros2Api;
            break;
        }
    }
}
exports.selectROSApi = selectROSApi;
//# sourceMappingURL=ros.js.map