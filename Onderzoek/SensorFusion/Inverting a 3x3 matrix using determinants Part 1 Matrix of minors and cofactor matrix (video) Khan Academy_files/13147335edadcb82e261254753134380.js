(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["13147335edadcb82e261254753134380"],{"3Ma3":function(e,t,r){"use strict";r("pNMO");r("4Brf");r("ma9I");r("TeQF");r("yXV3");r("2B1R");r("zKZe");r("5DmW");r("NBAS");r("FZtP");var l=c(r("q1tI"));var n=u(r("17x9"));var a=r("mR6N");var o=u(r("Pwnf"));var f=u(r("0cFD"));var i=u(r("x1xB"));var s=u(r("og3G"));var p=u(r("q+vY"));function u(e){return e&&e.__esModule?e:{default:e}}function c(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function d(t){for(var e=1;e<arguments.length;e++){var r=arguments[e]!=null?arguments[e]:{};var n=Object.keys(r);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(r).filter(function(e){return Object.getOwnPropertyDescriptor(r,e).enumerable}))}n.forEach(function(e){S(t,e,r[e])})}return t}function v(){v=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var n in r){if(Object.prototype.hasOwnProperty.call(r,n)){e[n]=r[n]}}}return e};return v.apply(this,arguments)}function b(e,t){if(e==null)return{};var r=m(e,t);var n,o;if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(o=0;o<a.length;o++){n=a[o];if(t.indexOf(n)>=0)continue;if(!Object.prototype.propertyIsEnumerable.call(e,n))continue;r[n]=e[n]}}return r}function m(e,t){if(e==null)return{};var r={};var n=Object.keys(e);var o,a;for(a=0;a<n.length;a++){o=n[a];if(t.indexOf(o)>=0)continue;r[o]=e[o]}return r}function y(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function h(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function g(e,t,r){if(t)h(e.prototype,t);if(r)h(e,r);return e}function O(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return T(e)}function T(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function w(e){w=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return w(e)}function j(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)P(e,t)}function P(e,t){P=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return P(e,t)}function S(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var i18n=r("HEOz");var E=function(e){j(t,e);function t(){y(this,t);return O(this,w(t).apply(this,arguments))}g(t,[{key:"render",value:function e(){var t=this.props,r=t.isTabSelected,n=t.trigger,o=b(t,["isTabSelected","trigger"]);return l.createElement("button",v({className:(0,a.css)(x.notAButton,x.tabTrigger,r?x.activeTab:x.inactiveTab)},o),n)}}]);return t}(l.Component);S(E,"propTypes",{trigger:n.default.any,isTabSelected:n.default.bool.isRequired});var k=function(e){j(n,e);function n(e){var t;y(this,n);t=O(this,w(n).call(this,e));S(T(t),"onTabChange",function(e){t.setState({currentTabKey:e})});S(T(t),"createRenderers",function(e){var t=function e(t,r){var n=t.key,o=t.trigger,a=t.content;return{key:n,renderTab:function e(t,r){return l.createElement(E,v({key:r.id,trigger:o,isTabSelected:t},r),o)},renderContent:function e(){return a}}};return e.map(t)});var r=t.createTabs(e);t.state={currentTabKey:r.length?r[0].key:undefined};return t}g(n,[{key:"createTabs",value:function e(t){var r=t.authorList,n=t.description,o=t.practiceContent,a=t.transcriptContent,i=t.augmentedTranscript;var s=[];if(n||r.length){var u=l.createElement("span",{"aria-label":i18n._("About this video")},i18n._("About"));s.push({key:"videoTabAbout",trigger:u,content:l.createElement(f.default,{authorList:r,description:n})})}if(a){s.push({key:"videoTabTranscript",trigger:i18n._("Transcript"),content:a})}if(i){var c=l.createElement("span",{"aria-label":i18n._("More transcript information")},i18n._("Augmented Transcript"));s.push({key:"videoTabAugmentedTranscript",trigger:c,content:l.createElement(p.default,{augmentedTranscript:i})})}if(o){s.push({key:"videoTabPractice",trigger:i18n._("Practice"),content:o})}return s}},{key:"render",value:function e(){var r=this;var t=this.createTabs(this.props);if(!t.length){return null}return l.createElement("div",{className:(0,a.css)(x.info)},l.createElement(s.default,{tabs:this.createRenderers(t),selectedTabKey:this.state.currentTabKey,onTabChanged:function e(t){return r.onTabChange(t)},id:"videoPageTabs"},function(e,t){return[e(),l.createElement("div",{className:(0,a.css)(x.separator),key:"separator"}),t(x.tabContent)]}))}}]);return n}(l.Component);S(k,"propTypes",{authorList:f.default.propTypes.authorList,description:n.default.string,practiceContent:n.default.node,transcriptContent:n.default.node,augmentedTranscript:n.default.string});S(k,"defaultProps",{authorList:[]});var x=a.StyleSheet.create({info:d({},i.default.contentPadding),activeTab:d({},i.default.navbar.activeTab,{color:o.default.blue,boxShadowColor:o.default.blue}),inactiveTab:d({},i.default.navbar.inactiveTab,{color:"inherit"}),tabContent:d({},i.default.mixins.padding(16,0,16,0),i.default.typography.bodySmall,{color:i.default.colors.gray17}),notAButton:{backgroundColor:"transparent",border:"none",padding:0,textAlign:"left",display:"inline-block",cursor:"pointer"},tabTrigger:d({},i.default.navbar.tab,{margin:0,zIndex:1,":not(:last-child)":{marginRight:32}}),separator:{backgroundColor:i.default.colors.gray85,height:1,position:"relative",top:-1}});e.exports=k},AeXU:function(e,t,r){"use strict";r("pNMO");r("4Brf");r("ma9I");r("TeQF");r("5DmW");r("NBAS");r("UxlC");r("SYor");r("FZtP");var O=v(r("q1tI"));var T=r("mR6N");var n=r("g8Be");var a=v(r("6qX+"));var w=p(r("KZFn"));var o=p(r("Q8Wn"));var u=p(r("nVHT"));var i=p(r("QCGh"));var c=p(r("UdSq"));var s=p(r("x1xB"));var j=p(r("ZpGe"));var P=p(r("kJvo"));var S=p(r("wlAN"));var E=p(r("7duV"));var k=p(r("HB80"));var l=p(r("3Ma3"));var d=r("jPIJ");var f=p(r("ClMb"));var x=p(r("iFiZ"));function p(e){return e&&e.__esModule?e:{default:e}}function v(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function b(t){for(var e=1;e<arguments.length;e++){var r=arguments[e]!=null?arguments[e]:{};var n=Object.keys(r);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(r).filter(function(e){return Object.getOwnPropertyDescriptor(r,e).enumerable}))}n.forEach(function(e){N(t,e,r[e])})}return t}function m(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function y(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function h(e,t,r){if(t)y(e.prototype,t);if(r)y(e,r);return e}function g(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return M(e)}function C(e){C=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return C(e)}function M(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function I(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)L(e,t)}function L(e,t){L=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return L(e,t)}function N(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var i18n=r("HEOz");var B=function(e){I(o,e);function o(){var e;var s;m(this,o);for(var t=arguments.length,r=new Array(t),n=0;n<t;n++){r[n]=arguments[n]}s=g(this,(e=C(o)).call.apply(e,[this].concat(r)));N(M(s),"state",{playerModel:undefined,playerState:undefined,readyToShowTranscripts:!!s.props.preloadedTranscript});N(M(s),"reportPageInteractive",function(){var e="video_page";a.reportPageInteractiveTiming(e);a.reportPageUsableTiming(e,true)});N(M(s),"maybeRenderExtraLink",function(){if(s.props.breadcrumbs&&s.props.breadcrumbs.length>0&&s.props.breadcrumbs[0].href==="/test-prep/sat"){return O.createElement(i.default,{href:"/sat"},i18n.doNotTranslate("Learn more about SAT Practice"))}return null});N(M(s),"handleModelUpdate",function(e){s.setState({playerModel:e})});N(M(s),"renderDiscussion",function(){var e=s.props,t=e.discussionContext,r=e.topic,n=e.isLoadingUserData,o=e.video;var a=s.state.playerModel;var i=null;if(!a){return i}if(n){i=O.createElement(u.default,null)}else{i=O.createElement(c.default,{clarificationsEnabled:o.clarificationsEnabled,contentKind:"Video",contentId:o.slug,videoModel:a,discussionContext:t,topicSlug:r?r.slug:"",onNextPage:function e(){}})}return O.createElement("div",{className:(0,T.css)(U.discussion)},i)});return s}h(o,[{key:"render",value:function e(){var r=this;var t=this.props,n=t.breadcrumbs,o=t.classUpsell,a=t.domain,i=t.mobileTutorialNav,s=t.navigateToNextItem,u=t.nextContentItem,c=t.preloadedTranscript,l=t.showEditorShortcuts,f=t.task,p=t.topic,d=t.video,v=t.attribution,b=t.shouldAutoplay;var m=this.state.playerModel;var y=d.id||"";var h="".concat(d.kind,":").concat(y);var g=this.props.video.conceptTagsInfo||[];return O.createElement("div",{className:"task-container",itemScope:true,itemType:"http://schema.org/VideoObject"},O.createElement("meta",{itemProp:"name",content:d.translatedTitle}),O.createElement("meta",{itemProp:"description",content:d.translatedDescription}),O.createElement("meta",{itemProp:"uploadDate",content:d.dateAdded}),O.createElement("link",{itemProp:"thumbnailUrl",href:d.imageUrl}),O.createElement(P.default,{navigateToNextItem:s,nextContentItem:u,onComponentInteractive:function e(){return r.reportPageInteractive()},onModelUpdate:this.handleModelUpdate,onStateChanged:function e(t){return r.setState({playerState:t})},task:f,topic:p,video:d,attribution:v,shouldAutoplay:b}),o&&O.createElement(S.default,{classUpsell:o}),O.createElement("div",{className:(0,T.css)(U.header)},O.createElement(w.default,{title:d.translatedTitle.trim().replace(/ ([^ ]+)$/," $1"),breadcrumbs:n,domain:a,showEditorShortcuts:l,editContentUrl:"/devadmin/content/videos/".concat(d.slug,"/").concat(y),standards:d.standards})),!!m&&O.createElement(F,{key:"extras_".concat(d.youtubeId||""),authorList:d.authorList||[],description:d.translatedDescriptionHtml||d.translatedDescription||"",domain:a,model:m,preloadedTranscript:c,youtubeId:d.youtubeId,augmentedTranscript:d.augmentedTranscript}),O.createElement("div",{className:(0,T.css)(U.sharingFooterWrapper)},O.createElement("div",{className:(0,T.css)(U.sharingFooter)},O.createElement(j.default,{domain:a,title:d.translatedTitle,image:d.imageUrl,contentDescriptor:h}))),i,this.renderDiscussion(),g&&g.length>0&&O.createElement(E.default,{domain:a,tags:d.conceptTagsInfo}),c&&O.createElement(x.default,{subtitles:c.subtitles,inlineStyles:R}),O.createElement(k.default,{extraLink:this.maybeRenderExtraLink(),license:d.kaUserLicense,youtubeId:d.youtubeId||""}))}}]);return o}(O.Component);var A=function(e){I(a,e);function a(){var e;var t;m(this,a);for(var r=arguments.length,n=new Array(r),o=0;o<r;o++){n[o]=arguments[o]}t=g(this,(e=C(a)).call.apply(e,[this].concat(n)));N(M(t),"handleSetTime",function(e){if(t.props.model.player&&t.props.model.player.seekTo){t.props.model.player.seekTo(e/1e3,true)}});return t}h(a,[{key:"render",value:function e(){return O.createElement(f.default,{open:true,currentTimeMs:this.props.currentTimeMs,subtitles:this.props.subtitles,onSetTime:this.handleSetTime,inlineStyles:R,domainColor:s.default.domainColors(this.props.domain).domain3})}}]);return a}(O.Component);var D=function(e){I(i,e);function i(e){var t;m(this,i);t=g(this,C(i).call(this,e));N(M(t),"_updateCurrentTime",function(){if(!t.getSubtitles()){return}var e=t.props.model.player;if(e&&e.getCurrentTime){t.setState({currentTimeMs:e.getCurrentTime()*1e3})}});var r={};var n=e.preloadedTranscript,o=e.youtubeId;if(n){var a=new d.VideoTranscript(b({},n,{youtubeId:o}));if(n.locale){r[n.locale]=a;r[""]=a}}t.state={currentTimeMs:0,subtitlesFetching:{},subtitlesLocale:"",videoTranscriptsByLocale:r};return t}h(i,[{key:"componentDidMount",value:function e(){var t=this.props.schedule;this.fetchTranscriptForLocale(o.default.kaLocale);t.interval(this._updateCurrentTime,250)}},{key:"fetchTranscriptForLocale",value:function e(r){var n=this;var t={};var o=false;var a=function e(t,r,n){var o={};o[r]=n;return b({},t,o)};var i=this.state.videoTranscriptsByLocale||{};var s=i[r];if(!s){var u=this.props.youtubeId;s=new d.VideoTranscript({youtubeId:u,locale:r});i=a(i,r,s);t.videoTranscriptsByLocale=i;o=true}var c=this.state.subtitlesFetching||{};if(!s.get("subtitles")&&!c[r]){var l=a(c,r,true);t.subtitlesFetching=l;o=true;var f=function e(){return n.setState({subtitlesFetching:a(n.state.subtitlesFetching,r,false)})};var p=function e(){if(r!=="en"){n.fetchTranscriptForLocale("en")}};i[r].fetch({success:function e(){f();var t=i[r].get("subtitles");if(t&&t.length){n.setState({subtitlesLocale:r})}else{p()}},error:function e(){f();p()}})}if(o){this.setState(t)}}},{key:"getSubtitles",value:function e(){var t=this.state.subtitlesLocale;var r=this.state.videoTranscriptsByLocale;var n=r[t];var o=n&&n.get("subtitles");return o&&o.length?o:null}},{key:"render",value:function e(){var t=this.props,r=t.authorList,n=t.description,o=t.domain,a=t.model,i=t.augmentedTranscript;var s=this.getSubtitles();var u=null;if(s){u=O.createElement(A,{currentTimeMs:this.state.currentTimeMs,domain:o,model:a,subtitles:s})}return O.createElement(l.default,{authorList:r,description:n,transcriptContent:u,augmentedTranscript:i})}}]);return i}(O.Component);var F=(0,n.withActionScheduler)(D);var R=T.StyleSheet.create({container:{maxHeight:250,overflowY:"scroll",position:"relative",WebkitOverflowScrolling:"touch"},sharingFooter:{borderTop:"1px solid ".concat(s.default.colors.gray85),paddingTop:15,paddingBottom:64},link:b({},s.default.typography.bodyXsmall,{display:"flex",paddingBottom:5,paddingTop:5,textDecoration:"none"}),dot:{fontSize:30,marginTop:-2,visibility:"hidden",minWidth:25},dotActive:{visibility:"visible"},time:{fontFamily:"inherit",fontWeight:"bold",minWidth:50}});var U=T.StyleSheet.create({header:{marginBottom:8},sharingFooterWrapper:b({},s.default.contentPadding),sharingFooter:{borderTop:"1px solid ".concat(s.default.colors.gray85),paddingTop:15,paddingBottom:64},discussion:{marginBottom:24,marginTop:28}});e.exports=B},ClMb:function(e,t,r){"use strict";r("x0AG");r("2B1R");r("zKZe");r("5DmW");var n=u(r("9/5/"));var s=u(r("TSYQ"));var o=u(r("fhzG"));var p=r("mR6N");var d=i(r("q1tI"));var a=u(r("17x9"));var v=r("Lrab");var b=u(r("Z+s4"));var m=u(r("C7Yw"));function i(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function u(e){return e&&e.__esModule?e:{default:e}}function y(){y=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var n in r){if(Object.prototype.hasOwnProperty.call(r,n)){e[n]=r[n]}}}return e};return y.apply(this,arguments)}var i18n=r("HEOz");function l(t,r,n){var o=t.scrollTop;var a=r-o;var i=performance.now();function s(e){e=Math.max(e-i,0);if(e>=n){t.scrollTop=r;return}t.scrollTop=o+a*e/n;window.requestAnimationFrame(s)}window.requestAnimationFrame(s)}var c=(0,o.default)({displayName:"VideoTranscriptView",propTypes:{currentTimeMs:a.default.number.isRequired,domainColor:a.default.string,inlineStyles:a.default.shape({container:a.default.objectOf(a.default.any),dot:a.default.objectOf(a.default.any),dotActive:a.default.objectOf(a.default.any),link:a.default.objectOf(a.default.any),text:a.default.objectOf(a.default.any),time:a.default.objectOf(a.default.any)}),onSetTime:a.default.func.isRequired,open:a.default.bool.isRequired,subtitles:a.default.arrayOf(a.default.shape({startTime:a.default.number,endTime:a.default.number,text:a.default.string})).isRequired},getInitialState:function e(){var t=this.props,r=t.currentTimeMs,n=t.subtitles;return{activeIndex:this.getActiveIndex(n,r),autoscroll:true}},componentDidMount:function e(){var t=this;this._isMounted=true;this.enableAutoscroll=(0,n.default)(function(){if(t.setState){t.setState({autoscroll:true})}},5e3)},UNSAFE_componentWillReceiveProps:function e(t){var r=t.subtitles,n=t.currentTimeMs;if(this.props.currentTimeMs!==n){this.setState({activeIndex:this.getActiveIndex(r,n)})}},shouldComponentUpdate:function e(t,r){var n=this.props,o=this.state;return o.autoscroll!==r.autoscroll||o.activeIndex!==r.activeIndex||n.open!==t.open||n.subtitles!==t.subtitles},componentDidUpdate:function e(t,r){var n=this.state,o=n.activeIndex,a=n.autoscroll;var i=o===r.activeIndex;if(!a||i||!this._isMounted){return}var s=this.refs.subtitleList;if(s.contains(document.activeElement)){return}var u=s.childNodes[o].offsetTop;var c=s.offsetHeight*.25;l(s,Math.max(0,u-c),200)},componentWillUnmount:function e(){if(this.enableAutoscroll){this.enableAutoscroll.cancel()}this._isMounted=false},_isMounted:false,getActiveIndex:function e(t,r){var n=t.findIndex(function(e){var t=e.startTime;return t>r});if(n===0){n=1}else if(n===-1){n=t.length}return n-1},handleSubtitleClick:function e(t){this.props.onSetTime(t)},render:function e(){var u=this;var t=this.props,c=t.domainColor,l=t.inlineStyles,r=t.subtitles;var f=this.state.activeIndex;var n=function e(t,r){var n;var o;var a;var i=r===f;if(l){n=(0,p.css)(h.row);o=(0,p.css)(l.time,h.time);a=(0,p.css)(l.text)}else{n=i&&"active";o="subtitle-time";a="subtitle-text"}var s={color:l&&i?c:"inherit"};return d.createElement("li",{className:n,key:r},l&&d.createElement("span",{"aria-hidden":"true",style:s,className:(0,p.css)(l.dot,i&&l.dotActive)},"•"),d.createElement(b.default,{onClick:function e(){return u.handleSubtitleClick(t.startTime)},style:h.segmentButton,inlineStyle:s},i&&d.createElement("span",{className:"sr-only"},i18n._("Current transcript segment: ")),d.createElement("span",{className:o},(0,v.formatTimeDisplay)(t.startTime/1e3)),d.createElement("span",{className:a},t.text)))};var o;var a;if(l){a=(0,p.css)(l.container)}else{o=(0,s.default)({"desktop-only":true,"new-subtitles-container":true,open:this.props.open,"subtitles-container":true,"min-contained-and-centered":true});a="subtitles"}var i={};if(m.default.touchevents){i={onTouchStart:function e(){u.enableAutoscroll.cancel();u.setState({autoscroll:false})},onTouchEnd:function e(){u.enableAutoscroll()}}}else{i={onMouseEnter:function e(){return u.setState({autoscroll:false})},onMouseLeave:function e(){return u.setState({autoscroll:true})}}}return d.createElement("div",{className:o},d.createElement("ul",y({className:a,itemProp:"transcript",ref:"subtitleList"},i),r.map(n)))}});var h=p.StyleSheet.create({segmentButton:{position:"relative",bottom:4,width:"80%"},time:{display:"inline-block",paddingLeft:8},row:{maxHeight:32,position:"relative",bottom:8}});e.exports=c},HB80:function(e,t,r){"use strict";r("pNMO");r("ma9I");r("TeQF");r("5DmW");r("NBAS");r("FZtP");var o=u(r("q1tI"));var n=s(r("6OHE"));var i=s(r("r+gr"));var a;function s(e){return e&&e.__esModule?e:{default:e}}function u(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function c(t){for(var e=1;e<arguments.length;e++){var r=arguments[e]!=null?arguments[e]:{};var n=Object.keys(r);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(r).filter(function(e){return Object.getOwnPropertyDescriptor(r,e).enumerable}))}n.forEach(function(e){h(t,e,r[e])})}return t}function l(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function f(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function p(e,t,r){if(t)f(e.prototype,t);if(r)f(e,r);return e}function d(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return b(e)}function v(e){v=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return v(e)}function b(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function m(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)y(e,t)}function y(e,t){y=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return y(e,t)}function h(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var g=r("mR6N"),O=g.StyleSheet,T=g.css;var w=r("whzC");var j=r("skGw"),P=j.khanFetch;var S=r("QCGh");var E=r("uEGL");var k=r("x1xB");var i18n=r("HEOz");var x=function(e){m(a,e);function a(){var e;var t;l(this,a);for(var r=arguments.length,n=new Array(r),o=0;o<r;o++){n[o]=arguments[o]}t=d(this,(e=v(a)).call.apply(e,[this].concat(n)));h(b(t),"state",{licenses:null});h(b(t),"_getLicenseInfo",function(){var e="/api/internal/licenses?v="+i.default.staticVersion;P(e).then(function(e){return e.json()}).then(function(e){if(t._isMounted){t.setState({licenses:e})}})});return t}p(a,[{key:"componentDidMount",value:function e(){this._isMounted=true;this._getLicenseInfo()}},{key:"componentWillUnmount",value:function e(){this._isMounted=false}},{key:"render",value:function e(){var t=this.state.licenses;var r=t&&t[this.props.license];var n=this.props.youtubeId&&w.safeLinkTo("https://www.youtube.com/watch?v=".concat(this.props.youtubeId));return o.createElement("div",null,r&&o.createElement("div",{className:T(C.videoFooterContainer)},n&&o.createElement(S,{href:n,style:[C.videoFooterLink,C.youtubeLink],target:"_blank"},i18n._("Video on YouTube")),o.createElement(S,{href:w.safeLinkTo(r.deedUrl),style:[C.videoFooterLink],target:"_blank"},r.fullname),this.props.extraLink&&o.createElement("span",{className:T(C.videoFooterLink)},this.props.extraLink)))}}]);return a}(o.Component);h(x,"defaultProps",{license:"cc-by-nc-sa"});var C=O.create({videoFooterContainer:c({},k.contentPadding,{paddingBottom:32}),videoFooterLink:{color:n.default.colors.gray55,display:"block",fontSize:12,fontStyle:"italic",lineHeight:"13px"},youtubeLink:(a={},h(a,E.mdOrLarger,{float:"right"}),h(a,E.smOrSmaller,{marginBottom:"12px"}),a)});e.exports=x},iFiZ:function(e,t,r){"use strict";r("pNMO");r("ma9I");r("TeQF");r("2B1R");r("5DmW");r("NBAS");r("FZtP");var s=o(r("q1tI"));var n;function o(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function a(t){for(var e=1;e<arguments.length;e++){var r=arguments[e]!=null?arguments[e]:{};var n=Object.keys(r);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(r).filter(function(e){return Object.getOwnPropertyDescriptor(r,e).enumerable}))}n.forEach(function(e){b(t,e,r[e])})}return t}function i(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function u(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function c(e,t,r){if(t)u(e.prototype,t);if(r)u(e,r);return e}function l(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return f(e)}function f(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function p(e){p=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return p(e)}function d(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)v(e,t)}function v(e,t){v=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return v(e,t)}function b(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var m=r("TSYQ");var y=r("17x9");var h=r("mR6N"),g=h.StyleSheet,O=h.css;var T=r("uEGL");var w=r("x1xB");var i18n=r("HEOz");var j=function(e){d(t,e);function t(){i(this,t);return l(this,p(t).apply(this,arguments))}c(t,[{key:"render",value:function e(){var t=this.props,r=t.inlineStyles,n=t.subtitles;var o;var a;if(r){a=O(r.container)}else{o=m({"desktop-only":true,"new-subtitles-container":true,open:true,"subtitles-container":true,"min-contained-and-centered":true});a="subtitles"}var i=n.map(function(e){return e.text}).join(" ");return s.createElement("div",{className:O(P.standaloneTranscript)},s.createElement("div",{className:o},s.createElement("h2",{className:O(P.title)},i18n._("Video transcript")),s.createElement("div",{className:a,itemProp:"transcript",ref:"subtitleList"},i)))}}]);return t}(s.Component);b(j,"propTypes",{inlineStyles:y.shape({container:y.objectOf(y.any)}),subtitles:y.arrayOf(y.shape({text:y.string})).isRequired});var P=g.create({standaloneTranscript:a({},w.typography.bodySmall,(n={maxWidth:688,margin:"auto",minHeight:0,marginBottom:32},b(n,T.smOrSmaller,{paddingLeft:16,paddingRight:16}),b(n,"paddingBottom",16),b(n,"borderBottom","1px solid ".concat(w.colors.gray85)),n)),title:a({},w.typography.labelLarge,{borderBottom:"1px solid ".concat(w.colors.gray85),paddingBottom:8,marginBottom:16})});e.exports=j},"q+vY":function(e,t,r){"use strict";r("5DmW");r("NBAS");var n=u(r("q1tI"));var o=r("Vxy9");var a=s(r("gE8/"));var i=r("JimW");function s(e){return e&&e.__esModule?e:{default:e}}function u(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function c(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function l(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function f(e,t,r){if(t)l(e.prototype,t);if(r)l(e,r);return e}function p(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return d(e)}function d(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function v(e){v=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return v(e)}function b(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)m(e,t)}function m(e,t){m=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return m(e,t)}var y=r("mR6N"),h=y.StyleSheet;var g=function(e){b(t,e);function t(){c(this,t);return p(this,v(t).apply(this,arguments))}f(t,[{key:"render",value:function e(){return n.createElement(i.View,{style:T.container},n.createElement(i.View,{style:T.innerContainer},n.createElement(o.Body,{style:{paddingRight:a.default.xxxSmall}},this.props.augmentedTranscript)),n.createElement(i.View,{style:T.topGradient}),n.createElement(i.View,{style:T.bottomGradient}))}}]);return t}(n.Component);var O=a.default.xSmall;var T=h.create({container:{position:"relative"},innerContainer:{maxHeight:250,overflowY:"scroll",position:"relative",WebkitOverflowScrolling:"touch",paddingBottom:O,paddingTop:O},bottomGradient:{position:"absolute",left:0,right:0,bottom:0,height:O,background:"linear-gradient(to top, rgba(255, 255, 255, 1) 0%, rgba(255, 255, 255, 0) 100%)"},topGradient:{position:"absolute",left:0,right:0,top:0,height:O,background:"linear-gradient(to top, rgba(255, 255, 255, 0) 0%, rgba(255, 255, 255, 1) 100%)"}});e.exports=g}}]);
//# sourceMappingURL=../../sourcemaps/en/13147335edadcb82e261254753134380.b36aa763ee7bd059da45.js.map