(()=>{"use strict";var e={591:e=>{e.exports=function(e,r,t){var n=(t=t||{}).marker||":",o=n.charCodeAt(0),a=n.length,i=t.validate||function(e){return e.trim().split(" ",2)[0]===r},c=t.render||function(e,t,n,o,a){return 1===e[t].nesting&&e[t].attrJoin("class",r),a.renderToken(e,t,n,o,a)};e.block.ruler.before("fence","container_"+r,(function(e,t,c,s){var l,d,p,u,f,m,v,b,h=!1,k=e.bMarks[t]+e.tShift[t],g=e.eMarks[t];if(o!==e.src.charCodeAt(k))return!1;for(l=k+1;l<=g&&n[(l-k)%a]===e.src[l];l++);if((p=Math.floor((l-k)/a))<3)return!1;if(l-=(l-k)%a,u=e.src.slice(k,l),f=e.src.slice(l,g),!i(f,u))return!1;if(s)return!0;for(d=t;!(++d>=c||(k=e.bMarks[d]+e.tShift[d])<(g=e.eMarks[d])&&e.sCount[d]<e.blkIndent);)if(o===e.src.charCodeAt(k)&&!(e.sCount[d]-e.blkIndent>=4)){for(l=k+1;l<=g&&n[(l-k)%a]===e.src[l];l++);if(!(Math.floor((l-k)/a)<p||(l-=(l-k)%a,(l=e.skipSpaces(l))<g))){h=!0;break}}return v=e.parentType,b=e.lineMax,e.parentType="container",e.lineMax=d,(m=e.push("container_"+r+"_open","div",1)).markup=u,m.block=!0,m.info=f,m.map=[t,d],e.md.block.tokenize(e,t+1,d),(m=e.push("container_"+r+"_close","div",-1)).markup=e.src.slice(k,l),m.block=!0,e.parentType=v,e.lineMax=b,e.line=d+(h?1:0),!0}),{alt:["paragraph","reference","blockquote","list"]}),e.renderer.rules["container_"+r+"_open"]=c,e.renderer.rules["container_"+r+"_close"]=c}}},r={};function t(n){var o=r[n];if(void 0!==o)return o.exports;var a=r[n]={exports:{}};return e[n](a,a.exports,t),a.exports}t.n=e=>{var r=e&&e.__esModule?()=>e.default:()=>e;return t.d(r,{a:r}),r},t.d=(e,r)=>{for(var n in r)t.o(r,n)&&!t.o(e,n)&&Object.defineProperty(e,n,{enumerable:!0,get:r[n]})},t.o=(e,r)=>Object.prototype.hasOwnProperty.call(e,r),t.r=e=>{"undefined"!=typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(e,"__esModule",{value:!0})};var n={};(()=>{t.r(n),t.d(n,{activate:()=>r});var e=t(591);function r(){const r="mermaid",t=[];return{extendMarkdownIt(n){n.use(e,r,{anyClass:!0,validate:e=>e.trim()===r,render:(e,r)=>{const n=e[r];var a="";if("container_mermaid_open"===n.type)for(var i=r+1;i<e.length;i++){const r=e[i];if(void 0===r||"container_mermaid_close"===r.type)break;a+=r.content,r.block&&r.nesting<=0&&(a+="\n"),r.tag="",r.type="inline",r.children=t}return 1===n.nesting?`<div class="mermaid">${o(a)}`:"</div>"}});const a=n.options.highlight;return n.options.highlight=(e,r)=>r&&r.match(/\bmermaid\b/i)?`<pre style="all:unset;"><div class="mermaid">${o(e)}</div></pre>`:a(e,r),n}}}const o=e=>e.replace(/\</g,"&lt;").replace(/\>/g,"&gt;")})(),module.exports=n})();
//# sourceMappingURL=index.js.map