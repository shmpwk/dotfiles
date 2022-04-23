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
const path = require("path");
const extension = require("../extension");
const telemetry = require("../telemetry-helper");
const preview_1 = require("./preview");
class URDFPreviewManager {
    constructor() {
        this._previews = [];
        this._activePreview = undefined;
    }
    setContext(context) {
        this._context = context;
    }
    refresh() {
        for (const preview of this._previews) {
            preview.refresh();
        }
    }
    preview(resource) {
        const reporter = telemetry.getReporter();
        reporter.sendTelemetryCommand(extension.Commands.PreviewURDF);
        if (URDFPreviewManager.handlesUri(resource)) {
            let preview = this.getExistingPreview(resource);
            if (preview) {
                preview.reveal();
            }
            else {
                preview = this.createNewPreview(this._context, resource);
            }
            preview.update(resource);
        }
    }
    get activePreviewResource() {
        return this._activePreview && this._activePreview.resource;
    }
    deserializeWebviewPanel(webview, state) {
        return __awaiter(this, void 0, void 0, function* () {
            if (state) {
                const preview = yield preview_1.default.revive(webview, this._context, state);
                this.registerPreview(preview);
            }
        });
    }
    getExistingPreview(resource) {
        return this._previews.find(preview => preview.matchesResource(resource));
    }
    createNewPreview(context, resource) {
        const preview = preview_1.default.create(context, resource);
        this._activePreview = preview;
        return this.registerPreview(preview);
    }
    registerPreview(preview) {
        this._previews.push(preview);
        preview.onDispose(() => {
            const existing = this._previews.indexOf(preview);
            if (existing === -1) {
                return;
            }
            this._previews.splice(existing, 1);
            if (this._activePreview === preview) {
                this._activePreview = undefined;
            }
        });
        preview.onDidChangeViewState(({ webviewPanel }) => {
            this._activePreview = webviewPanel.active ? preview : undefined;
        });
        return preview;
    }
    static handlesUri(uri) {
        let ext = path.extname(uri.fsPath);
        if (ext == ".xacro" ||
            ext == ".urdf") {
            return true;
        }
        return false;
    }
}
exports.default = URDFPreviewManager;
URDFPreviewManager.INSTANCE = new URDFPreviewManager();
//# sourceMappingURL=previewManager.js.map