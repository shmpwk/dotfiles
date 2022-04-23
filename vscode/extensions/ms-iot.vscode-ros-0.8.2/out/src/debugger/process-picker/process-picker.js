"use strict";
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.LocalProcessPicker = void 0;
const process_quick_pick = require("./process-quick-pick");
class LocalProcessPicker {
    constructor(quickPickItemsProvider) {
        this.quickPickItemsProvider = quickPickItemsProvider;
    }
    pick() {
        return process_quick_pick.showQuickPick(() => this.quickPickItemsProvider.getItems());
    }
}
exports.LocalProcessPicker = LocalProcessPicker;
//# sourceMappingURL=process-picker.js.map