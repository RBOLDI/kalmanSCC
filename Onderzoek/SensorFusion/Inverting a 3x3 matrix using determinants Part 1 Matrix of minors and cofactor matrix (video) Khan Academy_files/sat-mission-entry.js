(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["./javascript/app-shell-package/app-entry~./javascript/sat-mission-package/sat-mission-entry"],{"6XQz":function(e,t,r){"use strict";r("ma9I");r("5DmW");r("NBAS");r("07d7");r("5s+n");var a=u(r("q1tI"));var n=u(r("4PhQ"));var i=o(r("EZhD"));function o(e){return e&&e.__esModule?e:{default:e}}function u(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function l(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function c(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function s(e,t,r){if(t)c(e.prototype,t);if(r)c(e,r);return e}function d(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return h(e)}function f(e){f=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return f(e)}function h(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function p(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)b(e,t)}function b(e,t){b=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return b(e,t)}function v(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var y=r("mR6N"),g=y.css;var m=function(e){p(o,e);function o(){var e;var t;l(this,o);for(var r=arguments.length,n=new Array(r),i=0;i<r;i++){n[i]=arguments[i]}t=d(this,(e=f(o)).call.apply(e,[this].concat(n)));v(h(t),"state",{heightOfHeadersAboveContent:0});v(h(t),"_timeoutIdForUpdatingHeaderHeight",null);v(h(t),"_modifiedOuterWrapperMinHeight",false);return t}s(o,[{key:"componentDidMount",value:function e(){var t=this;if(this._shouldDisableFullPageBehavior()){return}this._scheduleHeaderHeightUpdate(true);Promise.all([r.e("corelibs"),r.e("corelibs-legacy"),r.e("shared"),r.e("shared-components"),r.e("wonder-blocks"),r.e("3fa99068de23e9956e10785c83deca6d"),r.e("8d7232af85394a26e0ab6c00da9e84ba"),r.e("6a3fa5d0e3e1ade7fafabbb88d4d7042"),r.e("326c71b53d46dcf9ce187853ae2bbcf4"),r.e("b463d1b36bc03e01e098b0025c19db16"),r.e("d2b510a52740e9565335ffb74418a12e"),r.e("246618b2fb0cda9f132c46b9925c444b"),r.e("0d7ec7d2eafb36f2c64ff30936848e13")]).then(r.t.bind(null,"3XV4",7)).then(function(e){return e.default}).then(function(e){e.on("show-urgent hide-urgent",t._handleUrgentNotifications,t)})}},{key:"componentWillUnmount",value:function e(){if(this._shouldDisableFullPageBehavior()){return}this._clearPendingTimeouts();if(this._modifiedOuterWrapperMinHeight){this._resetOuterWrapperMinHeight()}this._resetBodyScroll()}},{key:"_clearPendingTimeouts",value:function e(){var t=this._timeoutIdForUpdatingHeaderHeight;this._timeoutIdForUpdatingHeaderHeight=null;if(t!=null){clearTimeout(t)}}},{key:"_scheduleHeaderHeightUpdate",value:function e(t){var r=this;if(this._timeoutIdForUpdatingHeaderHeight!=null){return}this._timeoutIdForUpdatingHeaderHeight=setTimeout(function(){r._timeoutIdForUpdatingHeaderHeight=null;r.updateHeightOfHeadersAboveContent();if(t){r._removeBodyScroll()}},0)}},{key:"_handleUrgentNotifications",value:function e(){this._scheduleHeaderHeightUpdate()}},{key:"_removeBodyScroll",value:function e(){if(!document.body){n.default.error("Expected `document.body` to exist.",n.Errors.Internal);return}this._bodyOverflowYValueToReset=document.body.style.overflowY;document.body.style.overflowY="hidden"}},{key:"_resetBodyScroll",value:function e(){if(!document.body){n.default.error("Expected `document.body` to exist.",n.Errors.Internal);return}document.body.style.overflowY=this._bodyOverflowYValueToReset}},{key:"updateHeightOfHeadersAboveContent",value:function e(){if(!this._containerRef){return}var t=this._containerRef.getBoundingClientRect().top+window.scrollY;this.setState({heightOfHeadersAboveContent:t});this._removeOuterWrapperMinHeight()}},{key:"_removeOuterWrapperMinHeight",value:function e(){var t=document.getElementById("outer-wrapper");if(t){t.style.minHeight="initial";this._modifiedOuterWrapperMinHeight=true}}},{key:"_resetOuterWrapperMinHeight",value:function e(){var t=document.getElementById("outer-wrapper");if(t&&this._modifiedOuterWrapperMinHeight){t.style.minHeight=""}}},{key:"_shouldDisableFullPageBehavior",value:function e(){var t=this.props.forceFullPageBehavior;return(0,i.default)()&&!t}},{key:"render",value:function e(){var r=this;if(this._shouldDisableFullPageBehavior()){return this.props.children}var t=this.props,n=t.children,i=t.style;var o=this.state.heightOfHeadersAboveContent;return a.createElement("div",{style:{height:"calc(100vh - ".concat(o,"px)")},className:g(i),ref:function e(t){return r._containerRef=t}},n)}}]);return o}(a.Component);e.exports=m}}]);
//# sourceMappingURL=../../../../../../../sourcemaps/en/javascript/app-shell-package/app-entry~./javascript/sat-mission-package/sat-mission-entry.ffd923775e0f11038573.js.map