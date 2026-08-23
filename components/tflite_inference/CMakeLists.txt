idf_component_register(
    SRCS "tflite_inference.cc"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "../../model"
    PRIV_REQUIRES esp-tflite-micro
)