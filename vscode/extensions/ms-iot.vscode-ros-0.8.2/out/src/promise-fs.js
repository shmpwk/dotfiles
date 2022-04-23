"use strict";
// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.writeFile = exports.mkdir = exports.readdir = exports.readFile = exports.exists = void 0;
const fs = require("fs");
function call(fn, ...args) {
    return new Promise((c, e) => fn(...args, (err, res) => err ? e(err) : c(res)));
}
function exists(path) {
    return new Promise(c => fs.exists(path, c));
}
exports.exists = exists;
function readFile(filename, encoding) {
    return call(fs.readFile, ...arguments);
}
exports.readFile = readFile;
function readdir(path) {
    return call(fs.readdir, path);
}
exports.readdir = readdir;
function mkdir(path) {
    return call(fs.mkdir, path);
}
exports.mkdir = mkdir;
function writeFile(filename, data) {
    return call(fs.writeFile, filename, data);
}
exports.writeFile = writeFile;
//# sourceMappingURL=promise-fs.js.map