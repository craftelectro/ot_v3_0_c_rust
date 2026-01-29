cmake_minimum_required(VERSION 3.16)

idf_component_register(
    INCLUDE_DIRS "include"
)

set(RUST_TARGET "riscv32imac-unknown-none-elf")
set(CARGO_MANIFEST "${CMAKE_CURRENT_LIST_DIR}/rust/payload_parser/Cargo.toml")
set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo_target")
set(RUST_LIB "${CARGO_TARGET_DIR}/${RUST_TARGET}/release/libpayload_parser.a")

add_custom_command(
    OUTPUT ${RUST_LIB}
    COMMAND cargo build --release --target ${RUST_TARGET} --manifest-path ${CARGO_MANIFEST}
    COMMAND ${CMAKE_COMMAND} -E touch ${RUST_LIB}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/rust/payload_parser/src/lib.rs
            ${CMAKE_CURRENT_LIST_DIR}/rust/payload_parser/Cargo.toml
    COMMENT "Building Rust payload_parser (${RUST_TARGET})"
    VERBATIM
)

add_custom_target(rust_payload_build ALL DEPENDS ${RUST_LIB})

add_library(rust_payload_lib STATIC IMPORTED GLOBAL)
set_target_properties(rust_payload_lib PROPERTIES IMPORTED_LOCATION ${RUST_LIB})
add_dependencies(rust_payload_lib rust_payload_build)

add_dependencies(${COMPONENT_LIB} rust_payload_build)
target_link_libraries(${COMPONENT_LIB} INTERFACE rust_payload_lib)
