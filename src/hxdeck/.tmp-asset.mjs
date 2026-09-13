import cfFig from '/home/hx/Loli/code/HXLoLis/HXLoLi/src/hxdeck/figures/cf-gateway.ts';
console.log('typeof:', typeof cfFig);
console.log('keys:', Object.keys(cfFig||{}));
console.log('viewBox:', cfFig && cfFig.viewBox);
console.log('hasDefault:', !!(cfFig && cfFig.default));
console.log('default.viewBox:', cfFig && cfFig.default && cfFig.default.viewBox);
console.log('svg len:', cfFig && cfFig.svg ? cfFig.svg.length : 'N/A');