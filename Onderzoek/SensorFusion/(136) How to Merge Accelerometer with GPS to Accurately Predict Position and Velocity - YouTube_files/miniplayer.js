(function(g){var window=this;var O5=function(a,b){var c="ytp-miniplayer-button-bottom-right",d={D:"svg",P:{height:"18px",version:"1.1",viewBox:"0 0 22 18",width:"22px"},K:[{D:"g",P:{fill:"none","fill-rule":"evenodd",stroke:"none","stroke-width":"1"},K:[{D:"g",P:{transform:"translate(-1.000000, -3.000000)"},K:[{D:"polygon",P:{points:"0 0 24 0 24 24 0 24"}},{D:"path",P:{d:"M19,7 L5,7 L5,17 L19,17 L19,7 Z M23,19 L23,4.98 C23,3.88 22.1,3 21,3 L3,3 C1.9,3 1,3.88 1,4.98 L1,19 C1,20.1 1.9,21 3,21 L21,21 C22.1,21 23,20.1 23,19 Z M21,19.02 L3,19.02 L3,4.97 L21,4.97 L21,19.02 Z",
fill:"#fff","fill-rule":"nonzero"}}]}]}]},e="Videopagina openen";a.O().ea("kevlar_miniplayer_expand_top")&&(c="ytp-miniplayer-button-top-left",d={D:"svg",P:{height:"24px",version:"1.1",viewBox:"0 0 24 24",width:"24px"},K:[{D:"g",P:{fill:"none","fill-rule":"evenodd",stroke:"none","stroke-width":"1"},K:[{D:"g",P:{transform:"translate(12.000000, 12.000000) scale(-1, 1) translate(-12.000000, -12.000000) "},K:[{D:"path",P:{d:"M19,19 L5,19 L5,5 L12,5 L12,3 L5,3 C3.89,3 3,3.9 3,5 L3,19 C3,20.1 3.89,21 5,21 L19,21 C20.1,21 21,20.1 21,19 L21,12 L19,12 L19,19 Z M14,3 L14,5 L17.59,5 L7.76,14.83 L9.17,16.24 L19,6.41 L19,10 L21,10 L21,3 L14,3 Z",
fill:"#fff","fill-rule":"nonzero"}}]}]}]},e="Groter maken");g.U.call(this,{D:"button",ca:["ytp-miniplayer-expand-watch-page-button","ytp-button",c],P:{title:"{{title}}","data-tooltip-target-id":"ytp-miniplayer-expand-watch-page-button"},K:[d]});this.H=a;this.ma("click",this.onClick,this);this.la("title",g.oP(a,e,"i"));g.Te(this,g.bQ(b.wb(),this.element))},P5=function(a){g.U.call(this,{D:"div",
J:"ytp-miniplayer-ui"});this.il=!1;this.player=a;this.L(a,"minimized",this.Eh);this.L(a,"onStateChange",this.KO)},Q5=function(a){g.iO.call(this,a);
this.o=new P5(this.player);this.o.hide();g.VN(this.player,this.o.element,4);a.app.M.o&&(this.load(),g.J(a.getRootNode(),"ytp-player-minimized",!0))};
g.t(O5,g.U);O5.prototype.onClick=function(){this.H.oa("onExpandMiniplayer")};g.t(P5,g.U);g.k=P5.prototype;
g.k.show=function(){this.Fc=new g.Un(this.Nn,null,this);this.Fc.start();if(!this.il){this.tooltip=new g.xS(this.player,this);g.C(this,this.tooltip);g.VN(this.player,this.tooltip.element,4);this.tooltip.scale=.6;this.tb=new g.VP(this.player);g.C(this,this.tb);this.Si=new g.U({D:"div",J:"ytp-miniplayer-scrim"});g.C(this,this.Si);this.Si.aa(this.element);this.L(this.Si.element,"click",this.AC);var a=new g.U({D:"button",ca:["ytp-miniplayer-close-button","ytp-button"],P:{"aria-label":"Sluiten"},K:[g.MO()]});
g.C(this,a);a.aa(this.Si.element);this.L(a.element,"click",this.El);a=new O5(this.player,this);g.C(this,a);a.aa(this.Si.element);this.lj=new g.U({D:"div",J:"ytp-miniplayer-controls"});g.C(this,this.lj);this.lj.aa(this.Si.element);this.L(this.lj.element,"click",this.AC);var b=new g.U({D:"div",J:"ytp-miniplayer-button-container"});g.C(this,b);b.aa(this.lj.element);a=new g.U({D:"div",J:"ytp-miniplayer-play-button-container"});g.C(this,a);a.aa(this.lj.element);var c=new g.U({D:"div",J:"ytp-miniplayer-button-container"});
g.C(this,c);c.aa(this.lj.element);this.VC=new g.xR(this.player,this,!1);g.C(this,this.VC);this.VC.aa(b.element);b=new g.vR(this.player,this);g.C(this,b);b.aa(a.element);this.nextButton=new g.xR(this.player,this,!0);g.C(this,this.nextButton);this.nextButton.aa(c.element);this.gg=new g.mS(this.player,this);g.C(this,this.gg);this.gg.aa(this.Si.element);this.dc=new g.BR(this.player,this);g.C(this,this.dc);g.VN(this.player,this.dc.element,4);this.Gq=new g.U({D:"div",J:"ytp-miniplayer-buttons"});g.C(this,
this.Gq);g.VN(this.player,this.Gq.element,4);a=new g.U({D:"button",ca:["ytp-miniplayer-close-button","ytp-button"],P:{"aria-label":"Sluiten"},K:[g.MO()]});g.C(this,a);a.aa(this.Gq.element);this.L(a.element,"click",this.El);a=new g.U({D:"button",ca:["ytp-miniplayer-replay-button","ytp-button"],P:{"aria-label":"Sluiten"},K:[g.ZO()]});g.C(this,a);a.aa(this.Gq.element);this.L(a.element,"click",this.JM);this.L(this.player,"presentingplayerstatechange",this.Eb);this.L(this.player,"appresize",this.Oa);this.L(this.player,
"fullscreentoggled",this.Oa);this.Oa();this.il=!0}0!==this.player.getPlayerState()&&g.U.prototype.show.call(this);this.dc.show();this.player.unloadModule("annotations_module")};
g.k.hide=function(){this.Fc&&(this.Fc.dispose(),this.Fc=void 0);g.U.prototype.hide.call(this);this.player.app.M.o||(this.il&&this.dc.hide(),this.player.loadModule("annotations_module"))};
g.k.W=function(){this.Fc&&(this.Fc.dispose(),this.Fc=void 0);g.U.prototype.W.call(this)};
g.k.El=function(){this.player.stopVideo();this.player.oa("onCloseMiniplayer")};
g.k.JM=function(){this.player.playVideo()};
g.k.AC=function(a){if(a.target===this.Si.element||a.target===this.lj.element)g.R(this.player.O().experiments,"kevlar_miniplayer_play_pause_on_scrim")?g.NH(g.fE(this.player))?this.player.pauseVideo():this.player.playVideo():this.player.oa("onExpandMiniplayer")};
g.k.Eh=function(){g.J(this.player.getRootNode(),"ytp-player-minimized",this.player.app.M.o)};
g.k.Xc=function(){this.dc.Wb();this.gg.Wb()};
g.k.Nn=function(){this.Xc();this.Fc&&this.Fc.start()};
g.k.Eb=function(a){g.W(a.state,32)&&this.tooltip.hide()};
g.k.Oa=function(){this.dc.setPosition(0,g.EN(this.player).getPlayerSize().width,!1);this.dc.ix()};
g.k.KO=function(a){this.player.app.M.o&&(0===a?this.hide():this.show())};
g.k.wb=function(){return this.tooltip};
g.k.Mc=function(){return!1};
g.k.lf=function(){return!1};
g.k.Mf=function(){return!1};
g.k.Ew=function(){};
g.k.nj=function(){};
g.k.Im=function(){};
g.k.yj=function(){return null};
g.k.Xk=function(){return new g.Jh(0,0,0,0)};
g.k.handleGlobalKeyDown=function(){return!1};
g.k.handleGlobalKeyUp=function(){return!1};
g.k.Kl=function(a,b,c,d,e){var f=0,h=d=0,l=g.gi(a);if(b){c=g.ao(b,"ytp-prev-button")||g.ao(b,"ytp-next-button");var m=g.ao(b,"ytp-play-button"),n=g.ao(b,"ytp-miniplayer-expand-watch-page-button");c?f=h=12:m?(b=g.di(b,this.element),h=b.x,f=b.y-12):n&&(h=g.ao(b,"ytp-miniplayer-button-top-left"),f=g.di(b,this.element),b=g.gi(b),h?(h=8,f=f.y+40):(h=f.x-l.width+b.width,f=f.y-20))}else h=c-l.width/2,d=25+(e||0);b=g.EN(this.player).getPlayerSize().width;e=f+(e||0);l=g.Yd(h,0,b-l.width);e?(a.style.top=e+
"px",a.style.bottom=""):(a.style.top="",a.style.bottom=d+"px");a.style.left=l+"px"};
g.k.showControls=function(){};
g.k.Jh=function(){};
g.k.ih=function(){return!1};g.t(Q5,g.iO);Q5.prototype.create=function(){};
Q5.prototype.cg=function(){return!1};
Q5.prototype.load=function(){this.player.hideControls();this.o.show()};
Q5.prototype.unload=function(){this.player.showControls();this.o.hide()};g.tO.miniplayer=Q5;})(_yt_player);
