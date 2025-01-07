<template>
    <div><slot></slot></div>
</template>

<script>
    import { propsBinder, methodsBinder,findRealParent } from '../../../utils/utils.js';
    import Options from '../../../mixins/Options.js';
    // import {DomEvent, popup, extend } from 'leaflet';
    // import * as L from 'leaflet';
	import L from 'leaflet';
    const DomEvent = L.DomEvent;
    const popup = L.popup;
    const extend = L.extend;


    export default {
        name: 'LPopup',
        inject: ['lMap'],
        mixins: [Options],
        inheritAttrs: false,
        props: {
            addToMap:{
                type: Boolean,
                default: false
            },
            layer: {
                type: [Array, Object],
                default: null,
            },
            latlng:{
                custom:true,
                type: [Array, Object],
                default: null
            },
            content:{
                custom:true,
                type: [Object, String, Function]
            }
        },
        data() {
            return {
                originOptions: {},
                parentDom: null,
				_attrs:{},
				_listeners:{}
            };
        },
        methods: {
            setContent (newVal) {
                if (this.self && newVal !== null && newVal !== undefined) {
                    this.self.setContent(newVal);
                }
            },
            setLatlng(newVal){
                if (this.self && newVal !== null && newVal !== undefined) {
					// debugger
                    this.self.setLatLng(newVal);
					// this.self.openPopup(newVal);
					this.self.togglePopup();
                }
            },
            getParentDom(){
                let parentDom = this.addToMap ? this.lMap : this.$parent;
                if(!parentDom.self){
                    parentDom = findRealParent(this);
                }
                return parentDom;
            },
            // 初始化共有功能
            initFunction() {
                if (!this.self) return false;

                // 当前leaflet对象加入父级或根map中
                this.parentDom = this.getParentDom();
				// debugger
                if(this.parentDom.bindPopup){
                    this.parentDom.bindPopup(this.self);
                }else{
                    this.self.addTo(this.lMap.self)
                }
                // 继承当前leaflet对象的方法
                DomEvent.on(this.self, this._listeners);

                // 响应式参数处理
                propsBinder(this, this.self, this.$options.props);
                propsBinder(this, this.self, {}, this.options);

                this.$nextTick(() => {
                    this.ready = true;
                    // 是否支持原方法
                    this.useSelfMethods && methodsBinder(this, this.self);
                    // 监听当前组件实例化后
                    this.$emit('ready', this.self, this.selfOptions);
                });
            },
            initLeafletObject() {
				let _attrs={}, _listeners={};
				let attrsObj = this.$attrs;
				Object.keys(attrsObj).forEach(k=>{
					if(k && k.startsWith('on')){
						let newK = k.substring(2)
						newK = newK && newK.toLowerCase()
						_listeners[newK] = attrsObj[k];
					}else{
						_attrs[k] = attrsObj[k];
					}
				});
				this._attrs = _attrs;
				this._listeners = _listeners;

                this.selfOptions = extend(this.originOptions, this.options, _attrs);
                this.self = popup(this.selfOptions,this.layer);
                // .setLatLng(this.latlng).setContent(this.content);
                if (this.latlng !== undefined && this.latlng !== null) {
                    this.self.setLatLng(this.latlng);
                }
				this.self.setContent(this.content || this.$el);

                this.initFunction();
            },

        },
        mounted() {
            this.initLeafletObject();
        },
		beforeUnmount() {
            if (this.parentDom) {
                if (this.parentDom.unbindPopup) {
                    this.parentDom.unbindPopup();
                } else if (
                    this.parentDom.self &&
                    this.parentDom.self.unbindPopup
                ) {
                    this.parentDom.self.unbindPopup();
                }
            }
        },
    }
</script>
