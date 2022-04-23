"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.LocalProcessQuickPickItemsProvider = void 0;
// copied from https://github.com/microsoft/vscode-cpptools
/* tslint:disable */
const process_item = require("./process-entry");
class LocalProcessQuickPickItemsProvider {
    getItems() {
        return this.getInternalProcessEntries().then((processEntries) => {
            // localeCompare is significantly slower than < and > (2000 ms vs 80 ms for 10,000 elements)
            // We can change to localeCompare if this becomes an issue
            processEntries.sort((a, b) => {
                if (a.name === undefined) {
                    if (b.name === undefined) {
                        return 0;
                    }
                    return 1;
                }
                if (b.name === undefined) {
                    return -1;
                }
                const aLower = a.name.toLowerCase();
                const bLower = b.name.toLowerCase();
                if (aLower === bLower) {
                    return 0;
                }
                return aLower < bLower ? -1 : 1;
            });
            return processEntries.map((p) => process_item.createProcessQuickPickItem(p));
        });
    }
}
exports.LocalProcessQuickPickItemsProvider = LocalProcessQuickPickItemsProvider;
//# sourceMappingURL=local-process-items-provider.js.map