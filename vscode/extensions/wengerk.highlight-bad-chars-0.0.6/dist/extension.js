(()=>{"use strict";var e={904:(e,o)=>{Object.defineProperty(o,"__esModule",{value:!0}),o.default=["","","","","","","","","","","",""," ","¦","«","»","¼","½","¾","¿","¨","±"," ","᠎"," "," "," "," "," "," "," "," "," "," "," ","​","‍","–","—","\u2028"," "," ","　","\ufeff","￼","\0","","","","","","","","\b","\v","\f","","","","","","","","","","","","","","","","","","","","؜","‎","‏","‪","‫","‬","‭","‮","⁦","⁧","⁨","⁩","​",";","¸","ǀ","∣","­"]},496:e=>{e.exports=require("vscode")}},o={};function t(n){var i=o[n];if(void 0!==i)return i.exports;var r=o[n]={exports:{}};return e[n](r,r.exports,t),r.exports}var n={};(()=>{var e=n;Object.defineProperty(e,"__esModule",{value:!0}),e.activate=void 0;const o=t(496),i=t(904);function r(){const e=o.workspace.getConfiguration("highlight-bad-chars");let t=e.allowedUnicodeChars;const n=o.window.createTextEditorDecorationType(e.badCharDecorationStyle),r="["+i.default.join("")+(e.additionalUnicodeChars||[]).join("")+(e.asciiOnly?"-􏿿":"")+"]";return t&&t.length||(t=[]),{badCharDecorationType:n,charRegExp:r,allowedChars:t}}e.activate=function(e){let t=r();console.log("highlight-bad-chars decorator is activated with configuration",t);let n=null,i=o.window.activeTextEditor;function a(){n&&clearTimeout(n),n=setTimeout(c,500)}function c(){var e;if(!i)return;const n=new RegExp(t.charRegExp,"g"),r=i.document.getText(),a=[];let c;for(;c=n.exec(r);){if(t.allowedChars.includes(c[0]))continue;const n=i.document.positionAt(c.index),r=i.document.positionAt(c.index+c[0].length),s=null===(e=c[0].codePointAt(0))||void 0===e?void 0:e.toString(16).toUpperCase(),d={range:new o.Range(n,r),hoverMessage:`Bad char \\u${s} (${c[0]})`};a.push(d)}i.setDecorations(t.badCharDecorationType,a)}i&&a(),o.window.onDidChangeActiveTextEditor((e=>{i=e,e&&a()}),null,e.subscriptions),o.workspace.onDidChangeTextDocument((e=>{i&&e.document===i.document&&a()}),null,e.subscriptions),o.workspace.onDidChangeConfiguration((e=>{t=r(),console.log("highlight-bad-chars configuration updated",t)}),null,e.subscriptions)}})();var i=exports;for(var r in n)i[r]=n[r];n.__esModule&&Object.defineProperty(i,"__esModule",{value:!0})})();