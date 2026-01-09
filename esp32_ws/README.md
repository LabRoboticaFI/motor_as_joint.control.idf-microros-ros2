> [!TIP]
> Se altamente recomendable el uso nativo de Linux, ya que Docker en Windows tiende a relentizarse a lo largo del tiempo de desarrollo, prolongando el tiempo de desarrollo.

# Interfaces personalizadas

Para agregar interfaces personalizadas que pueda utilizar microros en la ESP32, se tiene que clonar el package en el directorio ```components\micro_ros_espidf_component\extra_packages```. Al agregar esto al componente se tiene que volver a compilar todo el componente y construir el ambiente de nuevo:

``` bash
rm -rf build
idf.py fullclean
idf.py clean-microros
idf.py menuconfig 
idf.py build
```

A partir de este momento ya se han creado los archivos fuente del paquete para poder ser utilizados en el código.