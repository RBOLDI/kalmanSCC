(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["875f0d644ab369028394d58d220ee06f"],{"6Uqd":function(e,t,r){"use strict";r("ma9I");r("fbCW");r("2B1R");r("5DmW");r("NBAS");Object.defineProperty(t,"__esModule",{value:true});t.default=void 0;var l=r("mR6N");var u=h(r("q1tI"));var i=d(r("ZpD3"));var c=d(r("k+eC"));var n=d(r("6OHE"));var a=d(r("SGU3"));var s=d(r("Q8Wn"));var f=d(r("JKja"));var p=d(r("9x/I"));var o=d(r("mkzS"));function d(e){return e&&e.__esModule?e:{default:e}}function h(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function g(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function v(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function y(e,t,r){if(t)v(e.prototype,t);if(r)v(e,r);return e}function b(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return w(e)}function m(e){m=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return m(e)}function w(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function O(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)k(e,t)}function k(e,t){k=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return k(e,t)}function S(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var i18n=r("HEOz");var C=["learner","teacher","parent"];var j={learner:i18n._("Learner"),teacher:i18n._("Teacher"),parent:i18n._("Parent")};var E=function(e){O(o,e);function o(){var e;var r;g(this,o);for(var t=arguments.length,n=new Array(t),a=0;a<t;a++){n[a]=arguments[a]}r=b(this,(e=m(o)).call.apply(e,[this].concat(n)));S(w(r),"state",{role:r.props.initialRole||"learner"});S(w(r),"updateRole",function(t){var e=C.find(function(e){return t===e});if(e&&r.state.role!==e){i.default.markMinorConversions([{id:"signup_switched_roles",extra:{originalRole:r.state.role,newRole:e}}]);r.props.onRoleChange(e);r.setState({role:e})}});S(w(r),"handleLoginClick",function(e){e.preventDefault();i.default.markMinorConversions([{id:"signup_login_link_clicked"}]);r.props.updateStep("login")});S(w(r),"handleJoinClick",function(){i.default.markMinorConversions([{id:"signup_enter_class_code_clicked"}])});return r}y(o,[{key:"renderRoleSelection",value:function e(){var t=this.props.referral;var r=this.state.role;var n=i18n._("Join Khan Academy as a");var a="";var o=true;var i=false;if(r==="parent"){switch(t){case"parent_approval_success":n=i18n._("Create your parent account");a=i18n._("We’ll keep you up-to-date with what your child is learning on Khan Academy.");o=false;i=true;break;case"parent_approval_failed":n=i18n._("Create your parent account");a=i18n._("We’ll walk you through creating your child’s account and then keep you up-to-date with what they’re learning on Khan Academy.");o=false;break}}return u.createElement("div",null,u.createElement("div",null,n,i&&u.createElement("span",{className:(0,l.css)(P.preambleOptional)},i18n._("Optional"))),a&&u.createElement("div",{className:(0,l.css)(P.preambleSubText)},a),o&&u.createElement(c.default,{selected:r,options:C.map(function(e){return{value:e,label:j[e]}}),onToggle:this.updateRole}))}},{key:"render",value:function e(){var t=this.state.role;return u.createElement(f.default,null,this.renderRoleSelection(),u.createElement(p.default,{continueUrl:this.props.continueUrl,googleContinueUrl:this.props.googleContinueUrl,role:t,birthdate:this.props.birthdate,onBirthdateChange:this.props.onBirthdateChange,updateStep:this.props.updateStep}),s.default.kaLocale==="en"&&t==="learner"&&u.createElement("div",{className:(0,l.css)(P.classCode)},u.createElement(a.default,{href:"/join",onClick:this.handleJoinClick},"Enter class code")),u.createElement("div",{className:(0,l.css)(P.login)},u.createElement(a.default,{href:"/login",onClick:this.handleLoginClick},i18n._("Already have an account?"))))}}]);return o}(u.Component);var P=l.StyleSheet.create({preambleOptional:{float:"right",fontSize:12,fontStyle:"italic",lineHeight:"22.5px",fontWeight:400},preambleSubText:{fontSize:15,fontWeight:400,marginTop:5,marginBottom:8},classCode:{marginTop:14,marginBottom:-14,textAlign:"center",fontSize:o.default.BODY_FONT_SIZE,color:n.default.colors.kaGreen},login:{marginTop:24,marginBottom:72,textAlign:"center",fontSize:o.default.BODY_FONT_SIZE,color:n.default.colors.kaGreen}});var R=E;t.default=R},B95q:function(e,t,r){"use strict";r("ma9I");r("5DmW");r("NBAS");r("hByQ");Object.defineProperty(t,"__esModule",{value:true});t.default=void 0;var p=l(r("q1tI"));var a=l(r("6qX+"));var n=i(r("/eLw"));var d=i(r("QuFw"));var h=i(r("ujpx"));var g=i(r("TkYW"));var v=i(r("zph5"));var y=r("WGxL");var b=i(r("6Uqd"));var m=i(r("radk"));var w=i(r("nLDZ"));var O=i(r("NV6Q"));var o=r("bHum");function i(e){return e&&e.__esModule?e:{default:e}}function l(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function u(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function c(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function s(e,t,r){if(t)c(e.prototype,t);if(r)c(e,r);return e}function f(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return S(e)}function k(e){k=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return k(e)}function S(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function C(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)j(e,t)}function j(e,t){j=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return j(e,t)}function E(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}var i18n=r("HEOz");var P=function(e){C(o,e);function o(){var e;var r;u(this,o);for(var t=arguments.length,n=new Array(t),a=0;a<t;a++){n[a]=arguments[a]}r=f(this,(e=k(o)).call.apply(e,[this].concat(n)));E(S(r),"state",{role:r.props.initialRole||"learner",birthdate:"",under13:r.props.initialRole!=="teacher"&&r.props.initialRole!=="parent"});E(S(r),"handleRoleChange",function(e){return r.setState({role:e})});E(S(r),"handleBirthdateChange",function(e,t){return r.setState({birthdate:e,under13:t})});return r}s(o,[{key:"componentDidMount",value:function e(){var t=this.props,r=t.markConversion,n=t.isInModal;switch(this.props.initialPurpose){case"signup":a.reportPageLifecycleTiming("signup_page",{interactive:true,usable:true});r({id:"pageview_signup"});break;case"login":a.reportPageLifecycleTiming("login_page",{interactive:true,usable:true});if(!n){r({id:"pageview_login"})}break}}},{key:"isForZendeskLogin",value:function e(){var t=(0,n.default)(window.top.location.search);return t.purpose==="zendesk"}},{key:"getContinueUrl",value:function e(){var t=this.props,r=t.continueUrl,n=t.alwaysContinue,a=t.initialRole;if(n){return r||""}switch(a){case"parent":return"/createchild";case"teacher":return"/students";default:return r||""}}},{key:"render",value:function e(){var l=this;var t=this.props,u=t.continueUrl,r=t.initialPurpose,n=t.isInModal,c=t.referral,a=t.showEmailForm,o=t.initialRole;var i=function e(t){switch(t){case"login":return p.createElement(g.default,{heading:l.isForZendeskLogin()?i18n._("To continue to the Help Center, "+"log in to Khan Academy"):i18n._("Good to see you again!"),detailContent:p.createElement(O.default,{purpose:"login"})});default:return p.createElement(w.default,{role:l.state.role,referral:l.props.referral||"default"})}};var s=function e(t,r){var n=l.state,a=n.role,o=n.birthdate,i=n.under13;switch(t){case"intro":return p.createElement(b.default,{continueUrl:u,googleContinueUrl:(0,y.makeGoogleContinueObject)(),initialRole:a,birthdate:o,referral:c,onRoleChange:l.handleRoleChange,onBirthdateChange:l.handleBirthdateChange,updateStep:r});case"identify":return p.createElement(m.default,{birthdate:o,under13:i,role:a,referral:c||"default",updateStep:r,continueUrl:u});case"login":return p.createElement(d.default,{continueUrl:u,googleContinueUrl:(0,y.makeGoogleContinueObject)(),role:a,updateStep:r})}};var f;if(a&&(o==="teacher"||o==="parent")){f="identify"}else if(r==="signup"){f="intro"}else{f="login"}return n?p.createElement(h.default,{embedded:true,initialStep:f,includePadding:false,detailScreen:i,actionScreen:s}):p.createElement(v.default,{initialStep:f,detailScreen:i,actionScreen:s})}}]);return o}(p.Component);E(P,"defaultProps",{isInModal:false});var R=function e(t){return{activity:o.enums.Activity.OTHER_ACTIVITY}};var T=(0,o.withConversionContext)(R)(P);var A=T;t.default=A},nLDZ:function(e,t,r){"use strict";r("pNMO");r("4Brf");r("fbCW");r("NBAS");Object.defineProperty(t,"__esModule",{value:true});t.default=void 0;var o=f(r("q1tI"));var n=s(r("mkzS"));var a=r("mR6N");var i=r("wINb");var l=s(r("TkYW"));var u=s(r("NV6Q"));var c=r("JimW");function s(e){return e&&e.__esModule?e:{default:e}}function f(e){if(e&&e.__esModule){return e}else{var t={};if(e!=null){for(var r in e){if(Object.prototype.hasOwnProperty.call(e,r)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,r):{};if(n.get||n.set){Object.defineProperty(t,r,n)}else{t[r]=e[r]}}}}t.default=e;return t}}function p(e,t,r){if(t in e){Object.defineProperty(e,t,{value:r,enumerable:true,configurable:true,writable:true})}else{e[t]=r}return e}function d(e,t){if(!(e instanceof t)){throw new TypeError("Cannot call a class as a function")}}function h(e,t){for(var r=0;r<t.length;r++){var n=t[r];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function g(e,t,r){if(t)h(e.prototype,t);if(r)h(e,r);return e}function v(e,t){if(t&&(typeof t==="object"||typeof t==="function")){return t}return y(e)}function y(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function b(e){b=Object.setPrototypeOf?Object.getPrototypeOf:function e(t){return t.__proto__||Object.getPrototypeOf(t)};return b(e)}function m(e,t){if(typeof t!=="function"&&t!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:true,configurable:true}});if(t)w(e,t)}function w(e,t){w=Object.setPrototypeOf||function e(t,r){t.__proto__=r;return t};return w(e,t)}var O=r("uEGL");var i18n=r("HEOz");var k=[{role:"learner",copy:[{referral:"default",heading:i18n._("A world class education for anyone, anywhere. 100% free."),description:[i18n._("Join Khan Academy to get personalized help with what you’re studying or to learn something completely new. We’ll save all of your progress.")]}]},{role:"teacher",copy:[{referral:"LearnStorm",heading:i18n._("Welcome!"),description:[i18n._("To enroll for LearnStorm, you’ll need to create a teacher account on Khan Academy. Once you create an account, we’ll take you through a short overview of our tools. Don’t forget to enroll for LearnStorm at the end of the overview.")]},{referral:"default",heading:i18n._("Help every student succeed with personalized practice. 100% free."),description:[o.createElement("ul",{key:0,style:{listStyle:"disc",paddingLeft:n.default.SUBHEADING_FONT_SIZE}},o.createElement("li",null,i18n._("Find standards-aligned content")),o.createElement("li",null,i18n._("Assign practice exercises, videos and articles")),o.createElement("li",null,i18n._("Track student progress")),o.createElement("li",null,i18n._("Join millions of teachers and students")))]}]},{role:"parent",copy:[{referral:"default",heading:i18n._("First, create your parent account."),description:[i18n._("After that, we'll take you through creating an account for your child."),i18n._("Khan Academy is a great way for your child to get help with what they’re learning in school or to learn something completely new.")]},{referral:"parent_approval_success",heading:i18n._("Thank you for approving your child's account! Next, create your parent account."),description:[]},{referral:"parent_approval_failed",heading:i18n._("Your child's account has expired."),description:[]}]}];var S=function(e){m(t,e);function t(){d(this,t);return v(this,b(t).apply(this,arguments))}g(t,[{key:"getRoleCopyFor",value:function e(t){var r=this;var n={heading:"",description:[]};var a=k.find(function(e){return e.role===t});if(!a){return n}var o=a.copy;return o.find(function(e){return e.referral===r.props.referral})||o.find(function(e){return e.referral==="default"})||n}},{key:"render",value:function e(){var t=this.props,r=t.role,n=t.referral;var a=this.getRoleCopyFor(r);return o.createElement(c.View,{style:n==="LearnStorm"?C.learnStormSignupDetails:null},o.createElement(l.default,{transitionKey:r,heading:a.heading,optionalDetailItems:a.description,detailContent:o.createElement(u.default,{purpose:"signup"})}))}}]);return t}(o.Component);var C=a.StyleSheet.create({learnStormSignupDetails:p({backgroundColor:"#044760",width:"100%",height:"100%",display:"flex",justifyContent:"center"},O.lgOrLarger,{flexDirection:"row",backgroundImage:(0,i.cssUrl)("/images/learn-storm-campaigns/learnstorm-signup-banner-2x.png"),backgroundRepeat:"no-repeat",backgroundPosition:"bottom left",backgroundSize:495,minHeight:675})});var j=S;t.default=j}}]);
//# sourceMappingURL=../../sourcemaps/en/875f0d644ab369028394d58d220ee06f.b08e3b8e13a213e4ae25.js.map