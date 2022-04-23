"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.RosDebugConfigurationProvider = void 0;
const path = require("path");
const vscode = require("vscode");
const ros_1 = require("../../../ros/ros");
// interact with the user to create a roslaunch or rosrun configuration
class RosDebugConfigurationProvider {
    provideDebugConfigurations(folder, token) {
        return __awaiter(this, void 0, void 0, function* () {
            const type = yield vscode.window.showQuickPick(["ROS: Launch", "ROS: Attach"], { placeHolder: "Choose a request type" });
            if (!type) {
                return [];
            }
            switch (type) {
                case "ROS: Launch": {
                    const packageName = yield vscode.window.showQuickPick(ros_1.rosApi.getPackageNames(), {
                        placeHolder: "Choose a package",
                    });
                    if (!packageName) {
                        return [];
                    }
                    const launchFiles = (yield ros_1.rosApi.findPackageLaunchFiles(packageName)).concat(yield ros_1.rosApi.findPackageTestFiles(packageName));
                    const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
                    const target = yield vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
                    const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
                    if (!launchFilePath) {
                        return [];
                    }
                    return [{
                            name: "ROS: Launch",
                            request: "launch",
                            target: `${launchFilePath}`,
                            launch: "[rviz, gz, gzclient, gzserver]",
                            type: "ros",
                        }];
                }
                case "ROS: Attach": {
                    return [{
                            name: "ROS: Attach",
                            request: "attach",
                            type: "ros",
                        }];
                }
            }
            return [];
        });
    }
}
exports.RosDebugConfigurationProvider = RosDebugConfigurationProvider;
//# sourceMappingURL=ros.js.map