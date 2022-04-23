"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.activate = void 0;
// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
const vscode = require("vscode");
const bad_characters_1 = require("./bad-characters");
function loadConfiguration() {
    const configObj = vscode.workspace.getConfiguration('highlight-bad-chars');
    const badCharDecorationStyle = configObj.badCharDecorationStyle;
    const additionalChars = configObj.additionalUnicodeChars;
    const asciiOnly = !!configObj.asciiOnly;
    let allowedChars = configObj.allowedUnicodeChars;
    const badCharDecorationType = vscode.window.createTextEditorDecorationType(badCharDecorationStyle);
    const charRegExp = '[' +
        bad_characters_1.default.join('') +
        (additionalChars || []).join('') +
        (asciiOnly ? '\u{0080}-\u{10FFFF}' : '') +
        ']';
    if (!allowedChars || !allowedChars.length) {
        allowedChars = [];
    }
    return {
        badCharDecorationType,
        charRegExp,
        allowedChars,
    };
}
// this method is called when vs code is activated
function activate(context) {
    let config = loadConfiguration();
    console.log('highlight-bad-chars decorator is activated with configuration', config);
    let timeout = null;
    let activeEditor = vscode.window.activeTextEditor;
    if (activeEditor) {
        triggerUpdateDecorations();
    }
    vscode.window.onDidChangeActiveTextEditor(editor => {
        activeEditor = editor;
        if (editor) {
            triggerUpdateDecorations();
        }
    }, null, context.subscriptions);
    vscode.workspace.onDidChangeTextDocument(event => {
        if (activeEditor && event.document === activeEditor.document) {
            triggerUpdateDecorations();
        }
    }, null, context.subscriptions);
    vscode.workspace.onDidChangeConfiguration(event => {
        config = loadConfiguration();
        console.log('highlight-bad-chars configuration updated', config);
    }, null, context.subscriptions);
    function triggerUpdateDecorations() {
        if (timeout) {
            clearTimeout(timeout);
        }
        timeout = setTimeout(updateDecorations, 500);
    }
    function updateDecorations() {
        var _a;
        if (!activeEditor) {
            return;
        }
        const regEx = new RegExp(config.charRegExp, 'g');
        const text = activeEditor.document.getText();
        const badChars = [];
        let match;
        // tslint:disable-next-line:no-conditional-assignment
        while (match = regEx.exec(text)) {
            if (config.allowedChars.includes(match[0])) {
                continue;
            }
            const startPos = activeEditor.document.positionAt(match.index);
            const endPos = activeEditor.document.positionAt(match.index + match[0].length);
            const codePoint = (_a = match[0].codePointAt(0)) === null || _a === void 0 ? void 0 : _a.toString(16).toUpperCase();
            const decoration = {
                range: new vscode.Range(startPos, endPos),
                hoverMessage: `Bad char \\u${codePoint} (${match[0]})`,
            };
            badChars.push(decoration);
        }
        activeEditor.setDecorations(config.badCharDecorationType, badChars);
    }
}
exports.activate = activate;
//# sourceMappingURL=extension.js.map