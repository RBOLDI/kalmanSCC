(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["6211350708fd65b7d7bbe1b0500e0a7c"],{"1+Ti":function(e,r,t){"use strict";t("pNMO");t("ma9I");t("TeQF");t("yXV3");t("+2oP");t("3KgV");t("5DmW");t("07d7");t("5s+n");t("FZtP");Object.defineProperty(r,"__esModule",{value:true});r.loadGoogleAuthApi=r.loginWithGoogle=r.GOOGLE_LOGIN_MUTATION_FOR_POSTLOGIN=r.GOOGLE_LOGIN_MUTATION=r.getErrorMessageByCode=void 0;var s=t("4PhQ");var f=t("o1mU");var n=a(t("lTCR"));var o=a(t("Q8Wn"));var i=a(t("whzC"));var d=t("Y8R6");var p=t("MVfM");var g=t("c+8j");function a(e){return e&&e.__esModule?e:{default:e}}function v(r){for(var e=1;e<arguments.length;e++){var t=arguments[e]!=null?arguments[e]:{};var n=Object.keys(t);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(t).filter(function(e){return Object.getOwnPropertyDescriptor(t,e).enumerable}))}n.forEach(function(e){l(r,e,t[e])})}return r}function l(e,r,t){if(r in e){Object.defineProperty(e,r,{value:t,enumerable:true,configurable:true,writable:true})}else{e[r]=t}return e}function h(e,r){if(e==null)return{};var t=u(e,r);var n,o;if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(o=0;o<i.length;o++){n=i[o];if(r.indexOf(n)>=0)continue;if(!Object.prototype.propertyIsEnumerable.call(e,n))continue;t[n]=e[n]}}return t}function u(e,r){if(e==null)return{};var t={};var n=Object.keys(e);var o,i;for(i=0;i<n.length;i++){o=n[i];if(r.indexOf(o)>=0)continue;t[o]=e[o]}return t}function c(){var r=b(['\n    mutation loginWithGoogleMutationForPostlogin(\n        $token: String!\n        $birthdate: String\n    ) {\n        loginWithGoogle(token: $token, birthdate: $birthdate) {\n            user {\n                id\n                kaid\n                canAccessDistrictsHomepage\n                isTeacher\n                hasUnresolvedInvitations\n                transferAuthUrl(pathname: "")\n                preferredKaLocale {\n                    id\n                    kaLocale\n                }\n            }\n            isFirstLogin\n            error {\n                code\n            }\n        }\n    }\n']);c=function e(){return r};return r}function y(){var r=b(['\n    mutation loginWithGoogleMutation(\n        $token: String!\n        $birthdate: String\n        $linkClever: Boolean\n        $cleverLibrary: Boolean\n        $role: UserRole\n    ) {\n        loginWithGoogle(\n            token: $token\n            birthdate: $birthdate\n            linkClever: $linkClever\n            cleverLibrary: $cleverLibrary\n            role: $role\n        ) {\n            user {\n                id\n                kaid\n                canAccessDistrictsHomepage\n                isTeacher\n                hasUnresolvedInvitations\n                transferAuthUrl(pathname: "")\n                preferredKaLocale {\n                    id\n                    kaLocale\n                }\n            }\n            isFirstLogin\n            error {\n                code\n            }\n        }\n    }\n']);y=function e(){return r};return r}function b(e,r){if(!r){r=e.slice(0)}return Object.freeze(Object.defineProperties(e,{raw:{value:Object.freeze(r)}}))}var i18n=t("HEOz");var O=function e(r){if(o.default.kaLocale!=="en"){return i18n._("Oops! Something went wrong. Please try again.")}switch(r){case"popup_blocked_by_browser":return i18n._("Your browser blocked a popup window required for logging in. Please click on the new Continue with Google button and try logging in again.");case"idpiframe_initialization_failed":return i18n._("Your browser blocked third-party cookies required for logging in. Please change your browser settings to allow cookies on khanacademy.org.");case"popup_closed_by_user":return i18n._("You closed the popup window before logging in. If that was a mistake, try logging in again.");case"ALREADY_LOGGED_IN":return i18n._("You are currently logged in to another Google account. Please log out of that account and try again.");case"INVALID_TOKEN":default:return i18n._("Oops! Something went wrong. Please try again.")}};r.getErrorMessageByCode=O;var w=(0,n.default)(y());r.GOOGLE_LOGIN_MUTATION=w;var C=(0,n.default)(c());r.GOOGLE_LOGIN_MUTATION_FOR_POSTLOGIN=C;var m=function e(i){var r=i.cleverLibrary,t=i.linkClever,n=i.role,a=i.signupCodes,o=i.skipPostlogin,l=h(i,["cleverLibrary","linkClever","role","signupCodes","skipPostlogin"]);var u=o?w:C;var c=o?v({},l,{cleverLibrary:r,linkClever:t,role:(0,g.roleToUserRole)(n)}):l;return(0,f.apolloMutate)(u,{variables:c}).then(function(e){if(e.data.loginWithGoogle.error){var r=e.data.loginWithGoogle.error.code;if(r==="SHADOWS_EXISTING_ACCOUNT"){(0,d.redirect)("/login/oauth/verify?oauth_type=google&google_access_token=".concat(i.token))}throw new s.KAError(e.data.loginWithGoogle.error.code,s.Errors.Internal)}var t=e.data.loginWithGoogle,n=t.user,o=t.isFirstLogin;if(a&&a.length>0){return(0,p.joinClassroomsPostlogin)(a).then(function(e){return{user:n,isFirstLogin:o,joinClassrooms:e}})}return{user:n,isFirstLogin:o}}).catch(function(e){if(e instanceof s.KAError){throw e}throw new s.KAError("Failed to login",s.Errors.Internal)})};r.loginWithGoogle=m;var k=function e(){if(window.gapi){return Promise.resolve(window.gapi)}return new Promise(function(e,r){var t=document.createElement("script");t.src=i.default.safeLinkTo("https://apis.google.com/js/api:client.js");t.onload=function(){e(window.gapi)};t.onerror=function(e){r(e)};document.body.appendChild(t)})};var U=function e(){return k().then(function(n){return new Promise(function(t,r){n.load("auth2",{callback:function e(){var r=n.auth2.init({client_id:"124072386181-eogtmmv0qose5ovudl946d83miv1ia89.apps.googleusercontent.com",cookie_policy:"single_host_origin"});t(r)},onerror:function e(){r(new s.KAError("auth2 api failed to load",s.Errors.Internal))}})})})};r.loadGoogleAuthApi=U},ANZR:function(e,r,t){"use strict";t("+2oP");t("3KgV");Object.defineProperty(r,"__esModule",{value:true});r.createReauthUrl=r.reauthWithThirdParty=r.THIRD_PARTY_REAUTH_MUTATION=void 0;var n=t("o1mU");var o=a(t("lTCR"));var i=a(t("gXb0"));function a(e){return e&&e.__esModule?e:{default:e}}function l(){var r=u(["\n    mutation reauthWithThirdPartyMutation(\n        $loginType: String!\n        $token: String!\n    ) {\n        reauthWithThirdParty(loginType: $loginType, token: $token) {\n            error {\n                code\n            }\n            user {\n                id\n                kaid\n            }\n        }\n    }\n"]);l=function e(){return r};return r}function u(e,r){if(!r){r=e.slice(0)}return Object.freeze(Object.defineProperties(e,{raw:{value:Object.freeze(r)}}))}var c=(0,o.default)(l());r.THIRD_PARTY_REAUTH_MUTATION=c;var s=function e(r,t){return(0,n.apolloMutate)(c,{variables:{loginType:r,token:t}}).then(function(e){if(e.errors&&e.errors.length>0){throw new Error("Failed to login")}if(e.data.reauthWithThirdParty.error){throw new Error(e.data.reauthWithThirdParty.error.code)}return{user:e.data.reauthWithThirdParty.user}})};r.reauthWithThirdParty=s;var f=function e(r){switch(r){case"google":return"you_can_now_login_with_google";case"facebook":return"you_can_now_login_with_facebook";case"apple":return"you_can_now_login_with_apple";case"clever":return"you_can_now_login_with_clever";case"clever_library":return"you_can_now_login_with_clever_library";case"clever_district":case"password":return"N/A";default:r;throw new Error("Unsupported third party login type: ".concat(r))}};var d=function e(r,t){switch(r){case"ACCOUNT_ALREADY_LINKED":if(t==="apple"){return"already_linked_an_apple_account"}if(t==="facebook"){return"already_linked_a_facebook_account"}return"account_already_linked";case"DIDNT_LOGIN_SUCCESSFULLY":return"didnt_login_successfully";case"REAUTH_REQUIRED":return"reauth_required";case"NOT_LOGGED_IN":return"didnt_login_successfully";default:return"link_account_error"}};var p=function e(r,t){var n=t?d(t,r):f(r);return"/settings/account?".concat((0,i.default)({messageKey:n}))};r.createReauthUrl=p},AxB7:function(e,r,t){"use strict";t("ma9I");t("5DmW");t("NBAS");Object.defineProperty(r,"__esModule",{value:true});r.default=void 0;var n=O(t("q1tI"));var o=t("Pk2J");var a=O(t("t1sY"));var l=t("1+Ti");var u=t("VmSZ");var c=t("ANZR");var s=t("sPtk");var f=b(t("RNI6"));var d=t("Y8R6");var p=O(t("4PhQ"));var i=b(t("4Du/"));var g=b(t("wF1t"));var v=t("YmN9");var h=b(t("etw9"));var y=b(t("i0nR"));function b(e){return e&&e.__esModule?e:{default:e}}function O(e){if(e&&e.__esModule){return e}else{var r={};if(e!=null){for(var t in e){if(Object.prototype.hasOwnProperty.call(e,t)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,t):{};if(n.get||n.set){Object.defineProperty(r,t,n)}else{r[t]=e[t]}}}}r.default=e;return r}}function w(e,r){if(!(e instanceof r)){throw new TypeError("Cannot call a class as a function")}}function C(e,r){for(var t=0;t<r.length;t++){var n=r[t];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function m(e,r,t){if(r)C(e.prototype,r);if(t)C(e,t);return e}function k(e,r){if(r&&(typeof r==="object"||typeof r==="function")){return r}return P(e)}function U(e){U=Object.setPrototypeOf?Object.getPrototypeOf:function e(r){return r.__proto__||Object.getPrototypeOf(r)};return U(e)}function P(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function I(e,r){if(typeof r!=="function"&&r!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(r&&r.prototype,{constructor:{value:e,writable:true,configurable:true}});if(r)T(e,r)}function T(e,r){T=Object.setPrototypeOf||function e(r,t){r.__proto__=t;return r};return T(e,r)}function E(e,r,t){if(r in e){Object.defineProperty(e,r,{value:t,enumerable:true,configurable:true,writable:true})}else{e[r]=t}return e}var j=t("ZpD3");var i18n=t("HEOz");var L=function(e){I(i,e);function i(){var e;var o;w(this,i);for(var r=arguments.length,t=new Array(r),n=0;n<r;n++){t[n]=arguments[n]}o=k(this,(e=U(i)).call.apply(e,[this].concat(t)));E(P(o),"state",{error:null,loading:true,loggingIn:false});E(P(o),"preloadGoogleAuthApi",function(){o.setState({loading:true});o._gapi=(0,l.loadGoogleAuthApi)().then(function(e){o.setState({loading:false});return e}).catch(function(e){if(e.error){o.setState({error:new Error(e.error),loading:false})}})});E(P(o),"handleError",function(e){var r=o.props.onError;p.default.error(e,p.Errors.Internal);if(r){var t=(0,l.getErrorMessageByCode)(e.message);r(t)}o.setState({error:null,loading:false,loggingIn:false})});E(P(o),"signIn",function(e){var r=o.props.forceSelectAccount?{prompt:"select_account"}:{};return e.signIn(r)});E(P(o),"handleClick",function(e){if(!o._gapi){return}if(o.state.error){o.handleError(o.state.error);return}o.setState({loading:true});var r=o.props.minorConversions;if(r){j.markMinorConversions(r)}o._gapi.then(function(e){return o.signIn(e)}).then(function(e){var r=e.getAuthResponse().id_token;o.setState({loggingIn:true});if(o.props.reauth){o.doReauth(r)}else{o.doLogin(r)}},function(e){if(e.error){o.handleError(new Error(e.error))}})});E(P(o),"getContinueUrl",function(e){if(o.props.flag){return e}return(0,f.default)({type:"google",role:o.props.role,linkClever:o.props.linkClever,cleverLibrary:o.props.cleverLibrary,classCodes:o.props.classCodes,continueUrl:e,reauth:o.props.reauth,isNewLogin:true})});E(P(o),"completeLogin",function(e){var r=(0,v.continueUrlForRole)(o.props.postLoginContinueUrl,o.props.googleContinueUrl,o.props.role,true);var t=(0,s.calculateNextUrl)({location:o.props.location,userData:e.user,loginType:"google",continueUrl:r,isFirstLogin:e.isFirstLogin,joinClassrooms:e.joinClassrooms,cleverLibrary:o.props.cleverLibrary,linkClever:o.props.linkClever});o.setCookies(e);var n=o.getContinueUrl(t);(0,d.redirect)(n)});E(P(o),"doLogin",function(e){var r=o.props.flag?{token:e,signupCodes:o.props.classCodes,birthdate:o.props.birthdate,cleverLibrary:o.props.cleverLibrary,linkClever:o.props.linkClever,role:o.props.role,skipPostlogin:true}:{token:e,signupCodes:o.props.classCodes,birthdate:o.props.birthdate,skipPostlogin:false};(0,l.loginWithGoogle)(r).then(function(r){var e=o.props.udi;if(e){(0,u.linkCurrentUserToUDIByUUID)({uuid:e,loginType:"google"}).then(function(e){if(e){o.completeLogin(r)}}).catch(o.handleError)}else{o.completeLogin(r)}}).catch(function(e){if(e.message==="SHADOWS_EXISTING_ACCOUNT"){o.setState({loggingIn:true})}else{p.default.error(e,p.Errors.Internal);o.handleError(e)}})});E(P(o),"doReauth",function(e){(0,c.reauthWithThirdParty)("google",e).then(function(e){var r=(0,c.createReauthUrl)("google");if(o.props.onSuccess){o.props.onSuccess(r)}else{(0,d.redirect)(r)}o.setState({loading:false})}).catch(function(e){p.default.error(e,p.Errors.Internal);var r=(0,c.createReauthUrl)("google",e.message);if(o.props.onReauthError){o.props.onReauthError(r)}else{(0,d.redirect)(r)}o.setState({loading:false})})});return o}m(i,[{key:"componentDidMount",value:function e(){this.preloadGoogleAuthApi()}},{key:"setCookies",value:function e(r){if(r.isFirstLogin){a.createCookie("first_login_".concat(r.user.kaid),Date.now())}else{a.createCookie("returning_login_".concat(r.user.kaid),Date.now())}}},{key:"render",value:function e(){var r=!this._gapi||this.state.loading||!!this.props.disabled||this.props.flagLoading;return n.createElement(h.default,{title:!this.state.loggingIn?i18n._("Continue with Google"):i18n._("Logging you in..."),iconComponent:y.default,disabled:r,onClick:this.handleClick,width:this.props.width,grow:this.props.grow,shrink:this.props.shrink,size:this.props.size})}}]);return i}(n.Component);E(L,"defaultProps",{forceSelectAccount:true});var A=(0,o.compose)(i.default,(0,g.default)({name:"skip_postlogin"}))(L);r.default=A},Rry3:function(e,r,t){"use strict";Object.defineProperty(r,"__esModule",{value:true});r.faceIconColor=void 0;var n=t("Pwnf");var o=function e(r){return r?function(e){return(0,n.fade)(e,.32)}:function(e){return e}};r.faceIconColor=o},VmSZ:function(e,r,t){"use strict";t("+2oP");t("3KgV");Object.defineProperty(r,"__esModule",{value:true});r.linkCurrentUserToUDIByUUID=r.LINK_CURRENT_USER_TO_UDI_BY_UUID_MUTATION=void 0;var n=a(t("lTCR"));var o=t("4PhQ");var i=t("o1mU");function a(e){return e&&e.__esModule?e:{default:e}}function l(){var r=u(["\n    mutation linkCurrentUserToUDIByUUIDMutation(\n        $uuid: ID!\n        $loginType: String!\n        $method: UserDistrictInfoActivationMethod\n    ) {\n        linkCurrentUserToUDIByUUID(\n            uuid: $uuid\n            loginType: $loginType\n            method: $method\n        ) {\n            succeeded\n            error {\n                code\n                debugMessage\n            }\n        }\n    }\n"]);l=function e(){return r};return r}function u(e,r){if(!r){r=e.slice(0)}return Object.freeze(Object.defineProperties(e,{raw:{value:Object.freeze(r)}}))}var c=(0,n.default)(l());r.LINK_CURRENT_USER_TO_UDI_BY_UUID_MUTATION=c;var s=function e(r){return(0,i.apolloMutate)(c,{variables:r}).then(function(e){var r=e.data.linkCurrentUserToUDIByUUID,t=r.succeeded,n=r.error;if(n){throw new o.KAError(n.debugMessage,o.Errors.Internal)}return t}).catch(function(e){if(e instanceof o.KAError){throw e}throw new o.KAError("Failed to link the current user to an UDI",o.Errors.Internal)})};r.linkCurrentUserToUDIByUUID=s},YmN9:function(e,r,t){"use strict";t("pNMO");t("ma9I");t("yXV3");t("+2oP");t("zKZe");t("5DmW");t("NBAS");Object.defineProperty(r,"__esModule",{value:true});r.default=r.continueUrlForRole=void 0;var a=c(t("q1tI"));var n=o(t("wF1t"));var h=t("vlHx");var y=o(t("RNI6"));var l=o(t("AxB7"));var i=o(t("etw9"));var u=o(t("i0nR"));function o(e){return e&&e.__esModule?e:{default:e}}function c(e){if(e&&e.__esModule){return e}else{var r={};if(e!=null){for(var t in e){if(Object.prototype.hasOwnProperty.call(e,t)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,t):{};if(n.get||n.set){Object.defineProperty(r,t,n)}else{r[t]=e[t]}}}}r.default=e;return r}}function s(){s=Object.assign||function(e){for(var r=1;r<arguments.length;r++){var t=arguments[r];for(var n in t){if(Object.prototype.hasOwnProperty.call(t,n)){e[n]=t[n]}}}return e};return s.apply(this,arguments)}function f(e,r){if(e==null)return{};var t=d(e,r);var n,o;if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(o=0;o<i.length;o++){n=i[o];if(r.indexOf(n)>=0)continue;if(!Object.prototype.propertyIsEnumerable.call(e,n))continue;t[n]=e[n]}}return t}function d(e,r){if(e==null)return{};var t={};var n=Object.keys(e);var o,i;for(i=0;i<n.length;i++){o=n[i];if(r.indexOf(o)>=0)continue;t[o]=e[o]}return t}function p(e,r){if(!(e instanceof r)){throw new TypeError("Cannot call a class as a function")}}function g(e,r){for(var t=0;t<r.length;t++){var n=r[t];n.enumerable=n.enumerable||false;n.configurable=true;if("value"in n)n.writable=true;Object.defineProperty(e,n.key,n)}}function v(e,r,t){if(r)g(e.prototype,r);if(t)g(e,t);return e}function b(e,r){if(r&&(typeof r==="object"||typeof r==="function")){return r}return w(e)}function O(e){O=Object.setPrototypeOf?Object.getPrototypeOf:function e(r){return r.__proto__||Object.getPrototypeOf(r)};return O(e)}function w(e){if(e===void 0){throw new ReferenceError("this hasn't been initialised - super() hasn't been called")}return e}function C(e,r){if(typeof r!=="function"&&r!==null){throw new TypeError("Super expression must either be null or a function")}e.prototype=Object.create(r&&r.prototype,{constructor:{value:e,writable:true,configurable:true}});if(r)m(e,r)}function m(e,r){m=Object.setPrototypeOf||function e(r,t){r.__proto__=t;return r};return m(e,r)}function k(e,r,t){if(r in e){Object.defineProperty(e,r,{value:t,enumerable:true,configurable:true,writable:true})}else{e[r]=t}return e}var U=t("ZpD3");var i18n=t("HEOz");var P="/signup/google";var I=function e(r,t,n,o){if(r&&(o||r.slice(0,2)!=="/?"&&r!=="/")){return r}if(typeof t==="string"){return t}if(typeof t==="object"){return t[n==="teacher"||n==="parent"?n:"default"]}return"/"};r.continueUrlForRole=I;var T=function(e){C(o,e);function o(){var e;var v;p(this,o);for(var r=arguments.length,t=new Array(r),n=0;n<r;n++){t[n]=arguments[n]}v=b(this,(e=O(o)).call.apply(e,[this].concat(t)));k(w(v),"handleClick",function(e){var r=v.props,t=r.googleContinueUrl,n=r.linkClever,o=r.cleverLibrary,i=r.postLoginContinueUrl,a=r.minorConversions,l=r.role,u=r.classCodes,c=r.birthdate,s=r.udi,f=r.onSuccess,d=r.reauth;e.preventDefault();if(a){U.markMinorConversions(a)}var p=I(i,t,l);var g=(0,y.default)({type:"google",role:l,linkClever:n,cleverLibrary:o,classCodes:u,birthdate:c,continueUrl:p,udi:s,reauth:d});(0,h.launchGoogleConnection)(P,g,f)});return v}v(o,[{key:"render",value:function e(){return a.createElement(i.default,{title:i18n._("Continue with Google"),iconComponent:u.default,disabled:this.props.disabled,onClick:this.handleClick,width:this.props.width,grow:this.props.grow,shrink:this.props.shrink,size:this.props.size})}}]);return o}(a.Component);var E=function e(r){var t=r.flag,n=r.flagLoading,o=r.flagError,i=f(r,["flag","flagLoading","flagError"]);if(t){return a.createElement(l.default,i)}return a.createElement(T,s({},i,{disabled:n}))};var j=(0,n.default)({name:"new_google_login"})(E);r.default=j},"c+8j":function(e,r,t){"use strict";Object.defineProperty(r,"__esModule",{value:true});r.roleToUserRole=void 0;var n=function e(r){switch(r){case"teacher":return"TEACHER";case"parent":return"PARENT";case"learner":default:return undefined}};r.roleToUserRole=n},etw9:function(e,r,t){"use strict";t("5DmW");Object.defineProperty(r,"__esModule",{value:true});r.default=void 0;var o=c(t("q1tI"));var i=t("JimW");var n=t("mR6N");var a=u(t("Pwnf"));var l=u(t("Nw73"));function u(e){return e&&e.__esModule?e:{default:e}}function c(e){if(e&&e.__esModule){return e}else{var r={};if(e!=null){for(var t in e){if(Object.prototype.hasOwnProperty.call(e,t)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,t):{};if(n.get||n.set){Object.defineProperty(r,t,n)}else{r[t]=e[t]}}}}r.default=e;return r}}var s=function e(r){switch(r){case"xsmall":return 24;case"small":return 32;case"large":return 48;case"xlarge":return 60;case"default":case null:case undefined:return 40;default:r;return 40}};var f=function e(r){if(r==="xsmall"){return 12}if(r==="small"){return 16}if(r==="large"){return 24}if(r==="xlarge"){return 30}return 20};var d=function e(r){var t=s(r.size);var n=f(r.size);return o.createElement(l.default,{colors:{backgroundColor:a.default.white,borderColor:a.default.offBlack64,textColor:a.default.offBlack},hoverColors:{backgroundColor:a.default.offBlack8,borderColor:a.default.offBlack64,textColor:a.default.offBlack},disabled:!!r.disabled,disabledStateIsOpaque:true,onClick:r.onClick,width:r.width,grow:r.grow,shrink:r.shrink,size:r.size,testId:r.testId},o.createElement(i.View,{style:[p.iconContainer,{height:t-2,width:t-2,padding:(t-n-2)/2}]},r.iconComponent!=null?o.createElement(r.iconComponent,{disabled:!!r.disabled}):null),r.title)};var p=n.StyleSheet.create({iconContainer:{position:"absolute",top:0,left:0}});var g=d;r.default=g},i0nR:function(e,r,t){"use strict";t("5DmW");Object.defineProperty(r,"__esModule",{value:true});r.default=void 0;var i=n(t("q1tI"));var a=t("Rry3");function n(e){if(e&&e.__esModule){return e}else{var r={};if(e!=null){for(var t in e){if(Object.prototype.hasOwnProperty.call(e,t)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,t):{};if(n.get||n.set){Object.defineProperty(r,t,n)}else{r[t]=e[t]}}}}r.default=e;return r}}var o=function e(r){var t=r.disabled,n=t===void 0?false:t;var o=(0,a.faceIconColor)(n);return i.createElement("svg",{viewBox:"0 0 24 24",fill:"none",xmlns:"http://www.w3.org/2000/svg"},i.createElement("path",{d:"M21.43 12.62C21.43 11.95 21.37 11.31 21.26 10.69H12.40V14.33H17.46C17.25 15.51 16.58 16.50 15.59 17.17V19.53H18.63C20.40 17.89 21.43 15.48 21.43 12.62Z",fill:o("#4285F4")}),i.createElement("path",{d:"M12.40 21.81C14.94 21.81 17.07 20.96 18.63 19.53L15.59 17.17C14.75 17.73 13.67 18.07 12.40 18.07C9.95 18.07 7.88 16.41 7.14 14.19H4.00V16.63C5.55 19.70 8.73 21.81 12.40 21.81Z",fill:o("#34A853")}),i.createElement("path",{d:"M7.14 14.19C6.95 13.63 6.85 13.02 6.85 12.40C6.85 11.78 6.95 11.18 7.14 10.62V8.18H4.00C3.36 9.45 3 10.89 3 12.40C3 13.92 3.36 15.36 4.00 16.63L7.14 14.19Z",fill:o("#FBBC05")}),i.createElement("path",{d:"M12.40 6.74C13.78 6.74 15.02 7.21 16.00 8.15L18.69 5.45C17.07 3.93 14.94 3 12.40 3C8.73 3 5.55 5.11 4.00 8.18L7.14 10.62C7.88 8.39 9.95 6.74 12.40 6.74Z",fill:o("#EA4335")}))};var l=o;r.default=l},sPtk:function(e,r,t){"use strict";t("pNMO");t("4Brf");t("ma9I");t("yq1k");t("4mDm");t("07d7");t("JTJg");t("3bBZ");t("Kz25");Object.defineProperty(r,"__esModule",{value:true});r.calculateNextUrl=r.maybeAddAuthTransfer=r.nextUrlAfterFirstTimeLogin=r.buildRedirectUrlFromJoinedClasses=void 0;var v=t("YyAa");var s=n(t("Q8Wn"));var f=n(t("Ssu9"));function n(e){return e&&e.__esModule?e:{default:e}}var h=function e(r){switch(r.code){case"UNAUTHORIZED":return r.classroomName?"/coaches?restrictedClassName=".concat(r.classroomName):"";case"UDI_NOT_FOUND":return"/coaches?districtClassCode=".concat(r.signupCode);case"CLASSROOM_NOT_FOUND":return"/coaches?failedCode=".concat(r.signupCode);case"DISTRICT_SYNCED_CLASSROOM":return"/coaches?failedCode=".concat(r.signupCode);case"CANNOT_ADD_COACH":return"/coaches?failedCode=".concat(r.signupCode);default:r.code;return""}};var y=function e(r){if(r.errors){var t=true;var n=false;var o=undefined;try{for(var i=r.errors[Symbol.iterator](),a;!(t=(a=i.next()).done);t=true){var l=a.value;var u=h(l);if(u!==""){return u}}}catch(e){n=true;o=e}finally{try{if(!t&&i.return!=null){i.return()}}finally{if(n){throw o}}}}if(r.studentLists){var c=true;var s=false;var f=undefined;try{for(var d=r.studentLists[Symbol.iterator](),p;!(c=(p=d.next()).done);c=true){var g=p.value;if(g.id){return"/?joinedClassCode="+g.signupCode}}}catch(e){s=true;f=e}finally{try{if(!c&&d.return!=null){d.return()}}finally{if(s){throw f}}}}return"/coaches"};r.buildRedirectUrlFromJoinedClasses=y;function o(e,r){if(!r){return e}return"".concat(e,"?continue=").concat(encodeURIComponent(r))}var b=function e(r,t){var n=t.includes("createchild");if(!n&&r.hasUnresolvedInvitations){return o("/createchild",t)}return o(t)};r.nextUrlAfterFirstTimeLogin=b;var O=function e(r,t,n){var o=t.preferredKaLocale&&t.preferredKaLocale.kaLocale;var i=s.default.kaLocale;var a=f.default.parse(r.hostname);if(a.lang==="www"&&o&&o!==i){var l=(0,v.validateContinueUrl)(t.transferAuthUrl,n);if(l!==n){var u=new URL(l,s.default.origin);var c=u.searchParams.has("continue");if(c){u.searchParams.set("continue",n)}return u.href}}return n};r.maybeAddAuthTransfer=O;var w=function e(r,t,n){if(n||r==="clever"||r==="clever_library"){return r==="clever_library"||t}return false};var i=function e(r){var t=r.location,n=r.userData,o=r.loginType,i=r.continueUrl,a=r.isFirstLogin,l=r.joinClassrooms,u=r.cleverLibrary,c=r.linkClever;var s="";var f=["apple","google","facebook"].includes(o)&&i==="/createchild";if(l){i=y(l)}var d=w(o,u,c);if(d){if(i===""){i="/"}var p=i.includes("?")?"&":"?";i+=p+"fromCleverLibrary=1"}var g=(0,v.validateContinueUrl)(i,"/");if(f){g="/"}if(a){if(n.canAccessDistrictsHomepage){g="/districts"}else if(n.isTeacher){g="/coach/welcome"}s=b(n,g)}else{s=g}s=O(t,n,s);return s};r.calculateNextUrl=i}}]);
//# sourceMappingURL=../../sourcemaps/en/6211350708fd65b7d7bbe1b0500e0a7c.4c3b5dca6da305f85549.js.map