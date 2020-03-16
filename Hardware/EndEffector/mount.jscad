// A basic end effector mount with a flat area to drill, glue, etc... an object
// Thanks to Dmitry Vasilev who shared this code. 
// It's easy to customize and to make an STL file in your browser for 3D printing. 
// View this file at:
// https://openjscad.org/#https://raw.githubusercontent.com/HaddingtonDynamics/Dexter/master/Hardware/EndEffector/mount.jscad
// Click the help tab on the left side of the screen, to see how to make an STL file. 

// Parameters

function getParameterDefinitions() {
    return [
        { name: 'height', type: 'int', initial: 30, caption: 'Height?' },
        { name: 'flat_area_height', type: 'int', initial: 15, caption: 'Flat Area Height?' },
        { name: 'strake_height', type: 'float', initial: 2.8, caption: 'Strake Height?' },
        { name: 'strake_width', type: 'float', initial: 5.8, caption: 'Strake Width?' },
        { name: 'in_body_thick', type: 'int', initial: 5, caption: 'Inner Body Thickness?' },
        { name: 'out_body_thick', type: 'int', initial: 14, caption: 'Outer Body Thickness?' },
    ];
}

const R1 = 37 / 2;
const R2 = 19 / 2;
const R3 = 14;
const R4 = 18;
const R5 = 16;


function main(params) {
    const height = params.height;
    const W = params.strake_width;
    const H = params.strake_height;
    const H2 = params.in_body_thick;
    const W2 = params.out_body_thick;
    const flat_area_height = params.flat_area_height;

    function sticks() {
        return union(
            stick(W,H).translate([R3,0,0]).rotateZ(-120),
            stick(W,H).translate([R3,0,0]).rotateZ(120),
            stick(W,H).translate([R3,0,0])
        )
    }

    function stick(W, H) {
          return cube({size: [H,W, height+2], center: [true,true,false]})
            .translate([0,0,-1])
    }

    function baseDiff() {
        return difference(
                cylinder({r: R4+H2/2, h: height+2, center: [true, true, false],fn: 32 }).translate([0,0,-1]),
                cylinder({r: R5-H2/2, h: height+4, center: [true, true, false],fn: 64}).translate([0,0,-2]),
                cube({size: [20,W2, height+4], center: [true,true,false]}).translate([R3,0,-2]).rotateZ(120),
                cube({size: [20,W2, height+4], center: [true,true,false]}).translate([R3,0,-2]).rotateZ(-120),
                cube({size: [20,W2, height+4], center: [true,true,false]}).translate([R3,0,-2])
            );
    }

    function base(R1, R2) {
          return difference(
            cylinder({r: R1, h: height, center: [true, true, false],fn: 128}),
            cylinder({r: R2, h: height+2, center: [true, true, false],fn: 64})
                .translate([0,0,-1])
          ).setColor([0, 1, 0, 1])
    }

    return [
        difference(
            union(
                cube({size: [14,30,height], center: [true,true,false]})
                    .translate([-R4+1,0,0]),

                difference(
                    cylinder({r: R1+4, h: 5, center: [true, true, false],fn: 128 }).setColor([0, 0.5, 0.5]),
                    cylinder({r: R2, h: 6, center: [true, true, false],fn: 64})
                ),
                difference(
                    base(R1, R2),
                    baseDiff(),
                    sticks()
                )
            ),
            cube({size: [14,16,height], center: [true,true,false]})
                .translate([-12,0,5]),

            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(-14),
            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(14),
            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(-120-14),
            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(-120+14),
            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(120-14),
            cylinder({r: 1.6, h: 7, center: [true, true, false],fn: 32}).translate([-R3-2,0,-1]).rotateZ(120+14),

            cube({size: [15,31,height], center: [true,true,false]})
                    .translate([-18.5,0, flat_area_height])
        )
    ]
}
