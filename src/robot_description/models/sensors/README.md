# Recursos visuais dos sensores

Quatro modelos genéricos originais criados para este repositório, com geometria
procedural em `tools/build_sensor_meshes.py` (na raiz do workspace).
Não são modelos oficiais de fabricantes.

O atlas `textures/sensor_atlas.png` foi gerado por IA para esta entrega. As quatro
regiões representam placa eletrônica, plástico escuro, plástico branco e acabamento
do lidar. Os materiais de lente, metal e contatos também usam propriedades
de superfície no Collada. Não são necessárias texturas remotas em execução.

Prompt usado na criação do atlas:

> Use case: scientific-educational. Asset type: a single UV texture atlas for lightweight 3D robotics sensor models in Gazebo Classic and RViz. Generate one square 1024 by 1024 opaque RGB texture sheet, edge-to-edge, exactly four equally sized square quadrants in a 2 by 2 grid, no margins, no dividers, no perspective, no background scene, no product rendering. Flat orthographic albedo colors only, even diffuse lighting with no cast shadows. Top-left quadrant: macro view of a dark green electronics PCB surface, delicate copper/gold traces, vias, tidy cream silkscreen component outlines and fine pads; no large chips because chips are separate 3D geometry. Top-right quadrant: matte charcoal black injection-molded plastic with a subtle fine stippled grain and restrained machining detail, a usable continuous material texture. Bottom-left quadrant: warm ivory-white GPS antenna enclosure plastic, finely textured matte surface, subtle realistic manufacturing detail, mostly uniform, no dirt. Bottom-right quadrant: dark graphite anodized sensor housing surface with small red safety bands near two edges and finely spaced vent-like grooves, suitable for a compact 2D lidar body. Purpose is recognizable realistic physical sensor materials; small details should stay crisp and believable. No company marks, no text, no numbers, no labels, no watermarks, no gradients representing directional illumination.

Mantenha os DAE em `meshes/` e o PNG em `textures/`, preservando a posição relativa.
As malhas usam metros e eixo vertical Z. Os frames dos sensores são definidos
no Xacro, independentemente dos detalhes visuais.
