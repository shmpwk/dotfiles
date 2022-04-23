"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.LocalProcessItemsProviderFactory = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const os = require("os");
const ps_provider = require("./process-items-provider-impl-ps");
const wmic_provider = require("./process-items-provider-impl-wmic");
class LocalProcessItemsProviderFactory {
    static Get() {
        if (os.platform() === 'win32') {
            return new wmic_provider.WmicProvider();
        }
        else {
            return new ps_provider.PsProvider();
        }
    }
}
exports.LocalProcessItemsProviderFactory = LocalProcessItemsProviderFactory;
//# sourceMappingURL=process-items-provider.js.map