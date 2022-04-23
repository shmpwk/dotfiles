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
const vscode = require("vscode");
const path = require("path");
const ros_1 = require("../ros/ros");
const utils_1 = require("../ros/utils");
const vscode_1 = require("vscode");
class URDFPreview {
    constructor(webview, context, resource) {
        this._onDisposeEmitter = new vscode.EventEmitter();
        this.onDispose = this._onDisposeEmitter.event;
        this._onDidChangeViewStateEmitter = new vscode.EventEmitter();
        this.onDidChangeViewState = this._onDidChangeViewStateEmitter.event;
        this._webview = webview;
        this._context = context;
        this._resource = resource;
        this._processing = false;
        let subscriptions = [];
        const templateFilename = this._context.asAbsolutePath("templates/preview.html");
        vscode.workspace.openTextDocument(templateFilename).then(doc => {
            var previewText = doc.getText();
            this._webview.webview.html = previewText;
            setTimeout(() => this.refresh(), 1000);
        });
        this._webview.onDidChangeViewState(e => {
            if (e.webviewPanel.active) {
                setTimeout(() => this.refresh(), 1000);
            }
            this._onDidChangeViewStateEmitter.fire(e);
        }, this, subscriptions);
        vscode.workspace.onDidSaveTextDocument(event => {
            if (event && this.isPreviewOf(event.uri)) {
                this.refresh();
            }
        }, this, subscriptions);
        this._webview.onDidDispose(() => {
            this.dispose();
        }, null, subscriptions);
        this._disposable = vscode_1.Disposable.from(...subscriptions);
    }
    get state() {
        return {
            resource: this.resource.toString()
        };
    }
    static create(context, resource) {
        // Create and show a new webview
        var editor = vscode.window.createWebviewPanel('urdfPreview', // Identifies the type of the webview. Used internally
        'URDF Preview', // Title of the panel displayed to the user
        vscode.ViewColumn.Two, // Editor column to show the new webview panel in.
        {
            enableScripts: true,
            retainContextWhenHidden: true
        });
        return new URDFPreview(editor, context, resource);
    }
    get resource() {
        return this._resource;
    }
    refresh() {
        return __awaiter(this, void 0, void 0, function* () {
            if (this._processing == false) {
                this.loadResource();
            }
        });
    }
    loadResource() {
        return __awaiter(this, void 0, void 0, function* () {
            this._processing = true;
            var urdfText;
            let ext = path.extname(this._resource.fsPath);
            if (ext == ".xacro") {
                try {
                    urdfText = yield (0, utils_1.xacro)(this._resource.fsPath);
                }
                catch (err) {
                    vscode.window.showErrorMessage(err.message);
                }
            }
            else {
                // at this point, the text document could have changed
                var doc = yield vscode.workspace.openTextDocument(this._resource.fsPath);
                urdfText = doc.getText();
            }
            var packageMap = yield ros_1.rosApi.getPackages();
            // replace package://(x) with fully resolved paths
            var pattern = /package:\/\/(.*?)\//g;
            var match;
            while (match = pattern.exec(urdfText)) {
                var packagePath = yield packageMap[match[1]]();
                if (packagePath.charAt(0) === '/') {
                    // inside of mesh re \source, the loader attempts to concatinate the base uri with the new path. It first checks to see if the
                    // base path has a /, if not it adds it.
                    // We are attempting to use a protocol handler as the base path - which causes this to fail.
                    // basepath - vscode-webview-resource:
                    // full path - /home/test/ros
                    // vscode-webview-resource://home/test/ros.
                    // It should be vscode-webview-resource:/home/test/ros.
                    // So remove the first char.
                    packagePath = packagePath.substr(1);
                }
                let normPath = path.normalize(packagePath);
                let vsPath = vscode.Uri.file(normPath);
                let newUri = this._webview.webview.asWebviewUri(vsPath);
                let hackThePath = newUri.toString().replace('https:', '');
                // HACKHACK - the RosWebTools will alwayse prefix the paths with a '/' if we don't pass a prefix.
                // to workaround this without changing RWT, we are stripping off the known protocol, and passing the
                // resulting path into RWT with that known prefix as an option. Internally it will see that there is a prefix
                // and combine them. 
                urdfText = urdfText.replace('package://' + match[1], hackThePath);
            }
            var previewFile = this._resource.toString();
            console.log("URDF previewing: " + previewFile);
            this._webview.webview.postMessage({ command: 'previewFile', previewFile: previewFile });
            this._webview.webview.postMessage({ command: 'urdf', urdf: urdfText });
            this._processing = false;
        });
    }
    static revive(webview, context, state) {
        return __awaiter(this, void 0, void 0, function* () {
            const resource = vscode.Uri.parse(state.previewFile);
            const preview = new URDFPreview(webview, context, resource);
            return preview;
        });
    }
    matchesResource(otherResource) {
        return this.isPreviewOf(otherResource);
    }
    reveal() {
        this._webview.reveal(vscode.ViewColumn.Two);
    }
    isPreviewOf(resource) {
        return this._resource.fsPath === resource.fsPath;
    }
    update(resource) {
        const editor = vscode.window.activeTextEditor;
        // If we have changed resources, cancel any pending updates
        const isResourceChange = resource.fsPath !== this._resource.fsPath;
        this._resource = resource;
        // Schedule update if none is pending
        this.refresh();
    }
    dispose() {
        this._disposable.dispose();
        this._onDisposeEmitter.fire();
        this._onDisposeEmitter.dispose();
        this._onDidChangeViewStateEmitter.dispose();
        this._webview.dispose();
    }
}
exports.default = URDFPreview;
//# sourceMappingURL=preview.js.map