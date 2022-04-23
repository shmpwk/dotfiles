"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.RosDebugSession = void 0;
const adapter = require("vscode-debugadapter");
class RosDebugSession extends adapter.DebugSession {
    shutdown() {
        super.shutdown();
    }
    launchRequest(response, request) {
        // launch requests are propagated to runtime-specific debugger extensions,
        // this is only a proxy session, self-terminate immediately
        this.shutdown();
    }
    attachRequest(response, request) {
        // attach requests are propagated to runtime-specific debugger extensions,
        // this is only a proxy session, self-terminate immediately
        this.shutdown();
    }
}
exports.RosDebugSession = RosDebugSession;
//# sourceMappingURL=debug-session.js.map