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
exports._createPackage = void 0;
const vscode = require("vscode");
const child_process = require("child_process");
const extension = require("../extension");
/**
 * Interacts with the user to run a create package command.
 */
function _createPackage(createPkgCommand, uri) {
    return __awaiter(this, void 0, void 0, function* () {
        const name = yield vscode.window.showInputBox({
            prompt: "Package name",
            validateInput: val => val.match(/^\w+$/) ? "" : "Invalid name",
        });
        if (!name) {
            return;
        }
        const dependencies = yield vscode.window.showInputBox({
            prompt: "Dependencies",
            validateInput: val => val.match(/^\s*(\w+\s*)*$/) ? "" : "Invalid dependencies",
        });
        if (typeof dependencies === "undefined") {
            return;
        }
        const cwd = typeof uri !== "undefined" ? uri.fsPath : `${extension.baseDir}/src`;
        const opts = { cwd, env: extension.env };
        child_process.exec(createPkgCommand(dependencies, name), opts, (err, stdout, stderr) => {
            if (!err) {
                vscode.workspace.openTextDocument(`${cwd}/${name}/package.xml`).then(vscode.window.showTextDocument);
            }
            else {
                let message = "Could not create package";
                let index = stderr.indexOf("error:");
                if (index !== -1) {
                    message += ": " + stderr.substr(index);
                }
                vscode.window.showErrorMessage(message);
            }
        });
    });
}
exports._createPackage = _createPackage;
//# sourceMappingURL=common.js.map