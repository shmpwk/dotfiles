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
exports.showQuickPick = exports.getExtensionFilePath = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const path = require("path");
const vscode = require("vscode");
const extension = require("../../extension");
function getExtensionFilePath(extensionfile) {
    return path.resolve(extension.extPath, extensionfile);
}
exports.getExtensionFilePath = getExtensionFilePath;
class RefreshButton {
    get iconPath() {
        const refreshImagePathDark = getExtensionFilePath(path.join("assets", "process-picker", "refresh_inverse.svg"));
        const refreshImagePathLight = getExtensionFilePath(path.join("assets", "process-picker", "refresh.svg"));
        return {
            dark: vscode.Uri.file(refreshImagePathDark),
            light: vscode.Uri.file(refreshImagePathLight)
        };
    }
    get tooltip() {
        return "Refresh process list";
    }
}
function showQuickPick(getAttachItems) {
    return __awaiter(this, void 0, void 0, function* () {
        return getAttachItems().then(processEntries => {
            return new Promise((resolve, reject) => {
                let quickPick = vscode.window.createQuickPick();
                quickPick.title = "Attach to process";
                quickPick.canSelectMany = false;
                quickPick.matchOnDescription = true;
                quickPick.matchOnDetail = true;
                quickPick.placeholder = "Select the process to attach to";
                quickPick.items = processEntries;
                quickPick.buttons = [new RefreshButton()];
                let disposables = [];
                quickPick.onDidTriggerButton(button => {
                    getAttachItems().then(processEntries => quickPick.items = processEntries);
                }, undefined, disposables);
                quickPick.onDidAccept(() => {
                    if (quickPick.selectedItems.length !== 1) {
                        reject(new Error("Process not selected"));
                    }
                    let selected = {
                        commandLine: quickPick.selectedItems[0].detail,
                        pid: quickPick.selectedItems[0].pid,
                    };
                    disposables.forEach(item => item.dispose());
                    quickPick.dispose();
                    resolve(selected);
                }, undefined, disposables);
                quickPick.onDidHide(() => {
                    disposables.forEach(item => item.dispose());
                    quickPick.dispose();
                    reject(new Error("Process not selected."));
                }, undefined, disposables);
                quickPick.show();
            });
        });
    });
}
exports.showQuickPick = showQuickPick;
//# sourceMappingURL=process-quick-pick.js.map