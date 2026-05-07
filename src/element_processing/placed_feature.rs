use crate::block_definitions::{Block, BlockWithProperties, COBBLESTONE_WALL, DARK_OAK_DOOR_LOWER, DARK_OAK_DOOR_UPPER, GRAY_CONCRETE, STONE};
use crate::element_processing::{
    advertising, amenities, barriers, buildings, doors, emergency, highways, historic, landuse,
    leisure, man_made, natural, power, railways, tourisms, water_areas, waterways,
};
use crate::osm_parser::{ProcessedElement, ProcessedMemberRole, ProcessedNode, ProcessedRelation, ProcessedWay};
use std::collections::{HashMap, HashSet};
use itertools::Itertools;
use crate::element_processing::landuse::LanduseType;
use crate::element_processing::leisure::generate_leisure;

#[derive(Debug, PartialEq)]
#[repr(i32)]
pub enum PlacedFeature {
    SimpleFeature {
        x: i32,
        z: i32,
        blocks: Vec<(i32, i32, i32, BlockWithProperties)>,
    },
    Highway {
        node_points: Vec<ProcessedNode>,
        surface: Vec<Block>,
        width: f32,
        lanes: i32,
        zebra_crossing: bool,
    },
    WayIdk {
        way: ProcessedWay,
    },
    Area {
        way_id: u64,
        nodes: Vec<ProcessedNode>,
        surface: Block,
    },
    Landuse {
        nodes: Vec<ProcessedNode>,
        r#type: LanduseType
    },
    Tree {
        x: i32,
        z: i32,
        tags: HashMap<String, String>
    }
}

impl PlacedFeature {

    /// Returns a queuable placed feature if the element
    /// should not be placed immediately, or places the element
    /// if it is an unlayered "base" such as `landuse` or `natural`.
    pub fn create_or_place(
        element: ProcessedElement,
        suppressed_building_outlines: &HashSet<u64>,
    ) -> Option<(i32, PlacedFeature)> {
        let layer = element
            .tags()
            .get("layer")
            .map(|l| l.parse::<i32>().unwrap_or(0))
            .unwrap_or(0);
        match element {
            ProcessedElement::Way(way) => {
                if way.tags.contains_key("building") || way.tags.contains_key("building:part") {
                    // Skip building outlines that are suppressed by building relations with parts.
                    // The individual building:part ways will render instead.
                    if suppressed_building_outlines.contains(&way.id) {
                        return None;
                    }
                    return Some((layer, buildings::placed_feature(way)));
                }
                if let Some(highway) = way.tags.get("highway") {
                    return Some((layer, highways::placed_feature(way, highway)));
                }
                if let Some(landuse) = way.tags.get("landuse") {
                    landuse::create_or_place_feature(way, landuse);
                    return None
                }
                if let Some(natural) = way.tags.get("natural") {
                    return Some((layer, natural::placed_feature(way, natural)));
                }
                if let Some(amenity) = way.tags.get("amenity") {
                    return Some((layer, amenities::placed_feature_way(way, amenity)));
                }
                if let Some(leisure) = way.tags.get("leisure") {
                    return Some((layer, leisure::placed_feature(way, leisure)));
                }
                if let Some(barrier) = way.tags.get("barrier") {
                    return Some((layer, barriers::placed_feature(way, barrier)));
                }
                None
                // if let Some(val) = way.tags.get("waterway") {
                //     if val == "dock" {
                //         // docks count as water areas
                //         water_areas::generate_water_area_from_way(&mut editor, way, &xzbbox);
                //     } else {
                //         waterways::generate_waterways(&mut editor, way);
                //     }
                // } else if way.tags.contains_key("bridge") {
                //     //bridges::generate_bridges(&mut editor, way, ground_level); // TODO FIX
                // } else if way.tags.contains_key("railway") {
                //     railways::generate_railways(&mut editor, way, &mut subway_points);
                // } else if way.tags.contains_key("roller_coaster") {
                //     railways::generate_roller_coaster(&mut editor, way);
                // } else if way.tags.contains_key("aeroway") || way.tags.contains_key("area:aeroway")
                // {
                //     highways::generate_aeroway(&mut editor, way, args);
                // } else if way.tags.get("service") == Some(&"siding".to_string()) {
                //     highways::generate_siding(&mut editor, way);
                // } else if way.tags.get("tomb") == Some(&"pyramid".to_string()) {
                //     historic::generate_pyramid(&mut editor, way, args, &flood_fill_cache);
                // } else if way.tags.contains_key("man_made") {
                //     man_made::generate_man_made(&mut editor, &element, args);
                // } else if way.tags.contains_key("power") {
                //     power::generate_power(&mut editor, &element);
                // } else if way.tags.contains_key("place") {
                //     landuse::generate_place(&mut editor, way, args, &flood_fill_cache);
                // }
            }
            ProcessedElement::Node(node) => {
                if node.tags.contains_key("door") || node.tags.contains_key("entrance") {
                    return Some((layer, PlacedFeature::SimpleFeature {
                        x: node.x,
                        z: node.z,
                        blocks: vec![(0, 0, 0, BlockWithProperties::simple(GRAY_CONCRETE)),
                                     (0, 1, 0, BlockWithProperties::simple(DARK_OAK_DOOR_LOWER)),
                                     (0, 2, 0, BlockWithProperties::simple(DARK_OAK_DOOR_UPPER)),
                        ],
                    }))
                } else if node.tags.contains_key("natural")
                    && node.tags.get("natural") == Some(&"tree".to_string())
                {
                    return Some((layer, PlacedFeature::Tree {
                        x: node.x,
                        z: node.z,
                        tags: node.tags.clone()
                    }))
                }
                if let Some(amenity) = node.tags.get("amenity") {
                    return Some((layer, amenities::placed_feature_node(node, amenity)));
                }
                if let Some(barrier) = node.tags.get("barrier") {
                    if barrier == "bollard" {
                        return Some((layer, PlacedFeature::SimpleFeature {
                            x: node.x,
                            z: node.z,
                            blocks: vec![(0, 1, 0, BlockWithProperties::simple(COBBLESTONE_WALL)),]
                        }));
                    } else if barrier == "block" {
                        return Some((layer, PlacedFeature::SimpleFeature {
                            x: node.x,
                            z: node.z,
                            blocks: vec![(0, 1, 0, BlockWithProperties::simple(STONE)),]
                        }));
                    }
                }
                if let Some(highway) = node.tags.get("highway") {
                    if let Some(feature) = highways::placed_feature_node(node, highway) {
                        return Some((layer, feature));
                    }
                }
                // if node.tags.contains_key("tourism") {
                //     tourisms::generate_tourisms(&mut editor, node);
                // } else if node.tags.contains_key("man_made") {
                //     man_made::generate_man_made_nodes(&mut editor, node);
                // } else if node.tags.contains_key("power") {
                //     power::generate_power_nodes(&mut editor, node);
                // } else if node.tags.contains_key("historic") {
                //     historic::generate_historic(&mut editor, node);
                // } else if node.tags.contains_key("emergency") {
                //     emergency::generate_emergency(&mut editor, node);
                // } else if node.tags.contains_key("advertising") {
                //     advertising::generate_advertising(&mut editor, node);
                // }
                None
            }
            ProcessedElement::Relation(rel) => {
                if rel.tags.get("type").is_some_and(|typ| typ == "multipolygon") {
                    create_or_place_multipolygon(rel, suppressed_building_outlines)
                }
            }
        }
    }
}

fn create_or_place_multipolygon(rel: ProcessedRelation, supressed_building_outlines: &HashSet<u64>) -> Option<(i32, PlacedFeature)> {
    for member in &rel.members {
        if member.role == ProcessedMemberRole::Outer {
            let way_with_rel_tags = ProcessedWay {
                id: member.way.id,
                nodes: member.way.nodes.clone(),
                tags: rel.tags.clone(),
            };
            if let Some(landuse) = rel.tags.get("landuse") {
                landuse::create_or_place_feature(way_with_rel_tags, landuse);
            }
        }
    }
    None
}
