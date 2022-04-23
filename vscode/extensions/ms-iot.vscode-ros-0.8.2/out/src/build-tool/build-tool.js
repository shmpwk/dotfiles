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
exports.isROSBuildTask = exports.determineBuildTool = exports.BuildTool = void 0;
const path = require("path");
const vscode = require("vscode");
const extension = require("../extension");
const pfs = require("../promise-fs");
const telemetry = require("../telemetry-helper");
const catkin = require("./catkin");
const catkin_tools = require("./catkin-tools");
const colcon = require("./colcon");
class BuildTool {
    static registerTaskProvider() {
        return this.current._registerTaskProvider();
    }
    static createPackage(context) {
        return __awaiter(this, void 0, void 0, function* () {
            const reporter = telemetry.getReporter();
            reporter.sendTelemetryCommand(extension.Commands.CreateCatkinPackage);
            return this.current._createPackage();
        });
    }
}
exports.BuildTool = BuildTool;
// tslint:disable-next-line: max-classes-per-file
class NotImplementedBuildTool extends BuildTool {
    _registerTaskProvider() {
        return null;
    }
    _createPackage() {
        return __awaiter(this, void 0, void 0, function* () {
            return;
        });
    }
}
// tslint:disable-next-line: max-classes-per-file
class CatkinMakeBuildTool extends BuildTool {
    static isApplicable(dir) {
        return __awaiter(this, void 0, void 0, function* () {
            return pfs.exists(`${dir}/.catkin_workspace`);
        });
    }
    _registerTaskProvider() {
        return [
            vscode.tasks.registerTaskProvider("catkin_make", new catkin.CatkinMakeProvider()),
            vscode.tasks.registerTaskProvider("catkin_make_isolated", new catkin.CatkinMakeIsolatedProvider()),
        ];
    }
    _createPackage() {
        return __awaiter(this, void 0, void 0, function* () {
            return catkin.createPackage();
        });
    }
}
// tslint:disable-next-line: max-classes-per-file
class CatkinToolsBuildTool extends BuildTool {
    static isApplicable(dir) {
        return __awaiter(this, void 0, void 0, function* () {
            return pfs.exists(`${dir}/.catkin_tools`);
        });
    }
    _registerTaskProvider() {
        return [vscode.tasks.registerTaskProvider("catkin", new catkin_tools.CatkinToolsProvider())];
    }
    _createPackage() {
        return __awaiter(this, void 0, void 0, function* () {
            return catkin_tools.createPackage();
        });
    }
}
// tslint:disable-next-line: max-classes-per-file
class ColconBuildTool extends BuildTool {
    static isApplicable(dir) {
        return __awaiter(this, void 0, void 0, function* () {
            return colcon.isApplicable(dir);
        });
    }
    _registerTaskProvider() {
        return [vscode.tasks.registerTaskProvider("colcon", new colcon.ColconProvider())];
    }
    _createPackage() {
        return __awaiter(this, void 0, void 0, function* () {
            // Do nothing.
            return;
        });
    }
}
BuildTool.current = new NotImplementedBuildTool();
/**
 * Determines build system and workspace path in use by checking for unique
 * auto-generated files.
 */
function determineBuildTool(dir) {
    return __awaiter(this, void 0, void 0, function* () {
        while (dir && path.dirname(dir) !== dir) {
            if (yield CatkinMakeBuildTool.isApplicable(dir)) {
                extension.setBaseDir(dir);
                BuildTool.current = new CatkinMakeBuildTool();
                return true;
            }
            else if (yield CatkinToolsBuildTool.isApplicable(dir)) {
                extension.setBaseDir(dir);
                BuildTool.current = new CatkinToolsBuildTool();
                return true;
            }
            else if (yield ColconBuildTool.isApplicable(dir)) {
                extension.setBaseDir(dir);
                BuildTool.current = new ColconBuildTool();
                return true;
            }
            dir = path.dirname(dir);
        }
        return false;
    });
}
exports.determineBuildTool = determineBuildTool;
/**
 * Check if a task belongs to our extension.
 * @param task Task to check
 */
function isROSBuildTask(task) {
    const types = new Set(["catkin", "catkin_make", "catkin_make_isolated", "colcon"]);
    const isRosTask = types.has(task.definition.type);
    const isBuildTask = vscode.TaskGroup.Build === task.group;
    return isRosTask && isBuildTask;
}
exports.isROSBuildTask = isROSBuildTask;
//# sourceMappingURL=build-tool.js.map