// U-Arm trigger: integrated SO-ARM-style finger ring
// Keeps the original mounting/transmission geometry; converts the distal paddle into a pull ring.
// Units: mm

base_file = "/mnt/data/config2 - trigger_short-1.STL";

// Distal paddle region of supplied U-Arm trigger is approximately:
// X 39.7..53.5, Y 288..312, Z 110..147.
ring_center = [46.61, 300.5, 132.0];
ring_thickness_x = 13.0;

// Finger opening (one finger / thin glove)
inner_width = 24.0;   // Y direction
inner_length = 30.0;  // Z direction
wall = 5.0;
outer_width = inner_width + 2*wall;      // 34
outer_length = inner_length + 2*wall;    // 40

$fn = 96;

module capsule2d(length, width) {
    r = width/2;
    d = max(0, length - width);
    hull() {
        translate([-d/2,0]) circle(r=r);
        translate([ d/2,0]) circle(r=r);
    }
}

module capsule_prism(center_xyz, length_z, width_y, thickness_x) {
    translate(center_xyz)
        rotate([0,90,0])
            linear_extrude(height=thickness_x, center=true, convexity=10)
                capsule2d(length_z, width_y);
}

difference() {
    union() {
        import(base_file, convexity=20);
        // Ring envelope overlaps the original distal paddle to form a single strong body.
        capsule_prism(ring_center, outer_length, outer_width, ring_thickness_x);
    }

    // Finger hole cuts through the merged distal paddle + ring.
    capsule_prism(ring_center, inner_length, inner_width, ring_thickness_x + 4.0);
}
