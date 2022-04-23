'use strict';
Object.defineProperty(exports, "__esModule", { value: true });
// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
const vscode = require("vscode");
const an = require("./runner");
const lint_1 = require("./lint");
const configuration_1 = require("./configuration");
let outputChannel;
let statusItem;
let timer;
let diagnosticCollection = vscode.languages.createDiagnosticCollection('cpplint');
// this method is called when your extension is activated
// your extension is activated the very first time the command is executed
function activate(context) {
    outputChannel = vscode.window.createOutputChannel('CppLint');
    // outputChannel.appendLine('CppLint is running.');
    // Use the console to output diagnostic information (console.log) and errors (console.error)
    // This line of code will only be executed once when your extension is activated
    console.log('Congratulations, your extension "cpplint" is now active!');
    loadConfigure();
    // The command has been defined in the package.json file
    // Now provide the implementation of the command with  registerCommand
    // The commandId parameter must match the command field in package.json
    let single = vscode.commands.registerCommand('cpplint.runAnalysis', runAnalysis);
    context.subscriptions.push(single);
    let whole = vscode.commands.registerCommand('cpplint.runWholeAnalysis', runWholeAnalysis);
    context.subscriptions.push(whole);
    vscode.workspace.onDidChangeConfiguration((() => loadConfigure()).bind(this));
}
exports.activate = activate;
function runAnalysis() {
    var edit = vscode.window.activeTextEditor;
    if (edit == undefined) {
        return Promise.reject("no edit opened");
    }
    outputChannel.show();
    outputChannel.clear();
    let start = 'CppLint started: ' + new Date().toString();
    outputChannel.appendLine(start);
    let result = an.runOnFile();
    outputChannel.appendLine(result);
    let end = 'CppLint ended: ' + new Date().toString();
    outputChannel.appendLine(end);
    // vscode.window.showInformationMessage(edit.document.uri.fsPath)
    return Promise.resolve();
}
function runWholeAnalysis() {
    outputChannel.show();
    outputChannel.clear();
    let start = 'CppLint started: ' + new Date().toString();
    outputChannel.appendLine(start);
    let result = an.runOnWorkspace();
    outputChannel.appendLine(result);
    let end = 'CppLint ended: ' + new Date().toString();
    outputChannel.appendLine(end);
    // vscode.window.showInformationMessage(edit.document.uri.fsPath)
    return Promise.resolve();
}
// this method is called when your extension is deactivated
function deactivate() {
    clearTimeout(timer);
    vscode.window.showInformationMessage("Cpplint deactivated");
}
exports.deactivate = deactivate;
function doLint() {
    if (vscode.window.activeTextEditor) {
        let language = vscode.window.activeTextEditor.document.languageId;
        if (configuration_1.ConfigManager.getInstance().isSupportLanguage(language)) {
            if (configuration_1.ConfigManager.getInstance().isSingleMode()) {
                lint_1.Lint(diagnosticCollection, false);
            }
            else {
                lint_1.Lint(diagnosticCollection, true);
            }
        }
    }
    clearTimeout(timer);
}
function startLint() {
    timer = global.setTimeout(doLint, 1.5 * 1000);
}
function startLint2() {
    timer = global.setTimeout(doLint, 500);
}
function loadConfigure() {
    configuration_1.ConfigManager.getInstance().initialize();
    if (configuration_1.ConfigManager.getInstance().isSingleMode()) {
        startLint2();
        vscode.window.onDidChangeActiveTextEditor((() => startLint2()).bind(this));
        vscode.workspace.onDidSaveTextDocument((() => startLint2()).bind(this));
    }
    else {
        // start timer to do workspace lint
        startLint();
        vscode.workspace.onDidSaveTextDocument((() => startLint()).bind(this));
    }
}
//# sourceMappingURL=extension.js.map