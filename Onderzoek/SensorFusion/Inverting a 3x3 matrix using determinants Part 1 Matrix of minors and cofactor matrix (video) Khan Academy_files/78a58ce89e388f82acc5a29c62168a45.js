(window["webpackJsonp"]=window["webpackJsonp"]||[]).push([["78a58ce89e388f82acc5a29c62168a45"],{VxSq:function(e,r,t){"use strict";t("pNMO");t("ma9I");t("TeQF");t("+2oP");t("3KgV");t("5DmW");t("FZtP");Object.defineProperty(r,"__esModule",{value:true});r.updateArticleProgress=void 0;var n=a(t("lTCR"));var s=a(t("x+rD"));var u=t("o1mU");var c=a(t("nSHN"));var l=o(t("4PhQ"));function o(e){if(e&&e.__esModule){return e}else{var r={};if(e!=null){for(var t in e){if(Object.prototype.hasOwnProperty.call(e,t)){var n=Object.defineProperty&&Object.getOwnPropertyDescriptor?Object.getOwnPropertyDescriptor(e,t):{};if(n.get||n.set){Object.defineProperty(r,t,n)}else{r[t]=e[t]}}}}r.default=e;return r}}function a(e){return e&&e.__esModule?e:{default:e}}function d(r){for(var e=1;e<arguments.length;e++){var t=arguments[e]!=null?arguments[e]:{};var n=Object.keys(t);if(typeof Object.getOwnPropertySymbols==="function"){n=n.concat(Object.getOwnPropertySymbols(t).filter(function(e){return Object.getOwnPropertyDescriptor(t,e).enumerable}))}n.forEach(function(e){i(r,e,t[e])})}return r}function i(e,r,t){if(r in e){Object.defineProperty(e,r,{value:t,enumerable:true,configurable:true,writable:true})}else{e[r]=t}return e}function f(){var r=p(["\n    mutation updateUserArticleProgress($input: UserArticleProgressInput!) {\n        updateUserArticleProgress(articleProgressUpdate: $input) {\n            actionResults {\n                tutorialNodeProgress {\n                    contentId\n                    progress\n                }\n                notificationsAdded {\n                    avatarParts\n                    badges\n                    continueUrl\n                    readable\n                    toast\n                    urgent\n                }\n                userProfile {\n                    countBrandNewNotifications\n                    countVideosCompleted\n                    points\n                    streakLastLength\n                    streakLastExtended\n                    streakLength\n                }\n            }\n            error {\n                code\n                debugMessage\n            }\n        }\n    }\n"]);f=function e(){return r};return r}function p(e,r){if(!r){r=e.slice(0)}return Object.freeze(Object.defineProperties(e,{raw:{value:Object.freeze(r)}}))}var g=(0,n.default)(f());var b=function e(r,t,n,o,a){s.default.getGAReferrer().then(function(e){return P(r,t,n,o,a,e)})};r.updateArticleProgress=b;var P=function e(r,t,n,o,a,s){var i={input:d({articleId:t,conversionId:r.toUpperCase(),articleSlug:n},o?{topicId:o}:null,{platform:"WEB",occurredAt:new Date,gaReferrer:s},a)};(0,u.apolloMutate)(g,{variables:i,module:"multithreaded"}).then(function(e){var r=e.data.updateUserArticleProgress;if(r.error){v(r.error.code)}else if(r.actionResults){c.default.respondToGraphQLAction(r.actionResults)}else{l.default.error("UpdateUserArticleProgress came back with both a null error and no actionResults.",l.Errors.Internal)}}).catch(function(e){l.default.error("Failed to update user article progress",l.Errors.Internal,e)})};var v=function e(r){l.default.error("Failed to update user article progress. Error returned from GraphQL mutation",l.Errors.Internal,null,{sentry:{extras:{errorCode:r},fingerprint:["updateUserArticleProgress",r]}})}}}]);
//# sourceMappingURL=../../sourcemaps/en/78a58ce89e388f82acc5a29c62168a45.98d4ddb31da02d75b73c.js.map