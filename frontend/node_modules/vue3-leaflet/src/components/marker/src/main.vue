<template>
    <div v-if="ready">
        <slot></slot>
    </div>
</template>

<script>
    import Options from '../../../mixins/Options.js';
    // import { marker, icon, extend } from 'leaflet';
    import iconUrl from 'leaflet/dist/images/marker-icon.png';
    import iconShadow from 'leaflet/dist/images/marker-shadow.png';
    // import * as L from 'leaflet';
	import L from 'leaflet';
    const marker = L.marker;
    const icon = L.icon;
    const divIcon = L.divIcon;
    const extend = L.extend;

    export default {
        name: 'LMarker',
        inject: ['lMap'],
        mixins: [Options],
        inheritAttrs: false,
        props: {
            latLng: {
                type: [Array, Object],
                default: null,
            },
            latlng: {
				custom:true,
                type: [Array, Object],
                default: null,
            },
			iconOptions:{
				custom:true,
				type: Object,
                default: null,
			},
			divIconOptions:{
				custom:true,
				type: Object,
                default: null,
			}
        },
        data() {
            return {
                originOptions: {}
            };
        },
        methods: {
			setLatlng(latlng){
				this.self.setLatLng(latlng);
			},
            initLeafletObject() {
                this.selfOptions = extend(this.originOptions, this.options, this.$attrs);
				if(this.iconOptions){
					this.selfOptions.icon =  icon(this.iconOptions);
				}else if(this.divIconOptions){
					this.selfOptions.icon =  divIcon(this.divIconOptions);
				}else{
					this.selfOptions.icon =  icon({
                        iconUrl: iconUrl,
                        shadowUrl: iconShadow,
                        iconAnchor: [12, 38],
                    });
				}
				let latlng = this.latLng || this.latlng
                this.self = marker(latlng, this.selfOptions);

                this.initFunction();
            },

        },
        mounted() {
            this.initLeafletObject();
        }
    }
</script>
