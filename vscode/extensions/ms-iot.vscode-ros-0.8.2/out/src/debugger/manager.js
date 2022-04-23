"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
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
exports.registerRosDebugManager = void 0;
const vscode = require("vscode");
const ros_provider = require("./configuration/providers/ros");
const attach_resolver = require("./configuration/resolvers/attach");
const ros1_launch_resolver = require("./configuration/resolvers/ros1/launch");
const ros2_launch_resolver = require("./configuration/resolvers/ros2/launch");
const extension = require("../extension");
class RosDebugManager {
    constructor() {
        this.configProvider = new ros_provider.RosDebugConfigurationProvider();
        this.attachResolver = new attach_resolver.AttachResolver();
        this.ros1LaunchResolver = new ros1_launch_resolver.LaunchResolver();
        this.ros2LaunchResolver = new ros2_launch_resolver.LaunchResolver();
    }
    provideDebugConfigurations(folder, token) {
        return __awaiter(this, void 0, void 0, function* () {
            return this.configProvider.provideDebugConfigurations(folder, token);
        });
    }
    resolveDebugConfigurationWithSubstitutedVariables(folder, config, token) {
        return __awaiter(this, void 0, void 0, function* () {
            if (config.request === "attach") {
                return this.attachResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config, token);
            }
            else if (config.request === "launch") {
                if ((typeof extension.env.ROS_VERSION === "undefined") || (extension.env.ROS_VERSION.trim() == "1")) {
                    return this.ros1LaunchResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config, token);
                }
                else {
                    return this.ros2LaunchResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config, token);
                }
            }
        });
    }
}
function registerRosDebugManager(context) {
    var rosProvider = new RosDebugManager();
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", rosProvider, vscode.DebugConfigurationProviderTriggerKind.Initial));
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", rosProvider, vscode.DebugConfigurationProviderTriggerKind.Dynamic));
}
exports.registerRosDebugManager = registerRosDebugManager;
//# sourceMappingURL=manager.js.map