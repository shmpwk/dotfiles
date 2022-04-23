"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.UnknownROS = void 0;
/**
 * Provides behavior for unknown ROS environment.
 */
class UnknownROS {
    setContext(context, env) {
    }
    getPackageNames() {
        console.error("Unknown ROS distro.");
        return;
    }
    getPackages() {
        console.error("Unknown ROS distro.");
        return;
    }
    getIncludeDirs() {
        console.error("Unknown ROS distro.");
        return;
    }
    getWorkspaceIncludeDirs(workspaceDir) {
        console.error("Unknown ROS distro.");
        return;
    }
    findPackageExecutables(packageName) {
        console.error("Unknown ROS distro.");
        return;
    }
    findPackageLaunchFiles(packageName) {
        console.error("Unknown ROS distro.");
        return;
    }
    findPackageTestFiles(packageName) {
        console.error("Unknown ROS distro.");
        return;
    }
    startCore() {
        console.error("Unknown ROS distro.");
        return;
    }
    stopCore() {
        console.error("Unknown ROS distro.");
        return;
    }
    getCoreStatus() {
        console.error("Unknown ROS distro.");
        return;
    }
    rosdep() {
        console.error("Unknown ROS distro.");
        return;
    }
    activateCoreMonitor() {
        console.error("Unknown ROS distro.");
        return;
    }
    showCoreMonitor() {
        console.error("Unknown ROS distro.");
        return;
    }
    activateRosrun(packageName, executableName, argument) {
        console.error("Unknown ROS distro.");
        return;
    }
    activateRoslaunch(launchFilepath, argument) {
        console.error("Unknown ROS distro.");
        return;
    }
    activateRostest(launchFilepath, argument) {
        console.error("Unknown ROS distro.");
        return;
    }
}
exports.UnknownROS = UnknownROS;
//# sourceMappingURL=unknown-ros.js.map