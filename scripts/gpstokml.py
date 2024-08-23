import zipfile
from xml.dom.minidom import Document

# gdal环境可用
# 将 2维 经纬度坐标转为 KML格式，在Google Earth 上进行展示

# 解析GPS点
def parse_gps_points(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            lat, lon = line.strip().split(',')
            points.append((float(lat), float(lon)))
    return points

# 创建KML文档
def create_kml(points):
    doc = Document()

    kml = doc.createElement('kml')
    kml.setAttribute('xmlns', 'http://www.opengis.net/kml/2.2')
    doc.appendChild(kml)

    document = doc.createElement('Document')
    kml.appendChild(document)

    for lat, lon in points:
        placemark = doc.createElement('Placemark')
        document.appendChild(placemark)

        point = doc.createElement('Point')
        placemark.appendChild(point)

        coordinates = doc.createElement('coordinates')
        coordinates.appendChild(doc.createTextNode(f'{lon},{lat},0'))
        point.appendChild(coordinates)

    return doc.toprettyxml(indent="  ")

# 将KML文件压缩为KMZ文件
def create_kmz(kml_data, kmz_path):
    with zipfile.ZipFile(kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.writestr('doc.kml', kml_data)

# 主函数
def main():
    txt_file_path = '../log/gps_points.txt'  # 你的GPS点文本文件路径
    kmz_file_path = '../log/gps_points.kmz'  # 输出的KMZ文件路径

    gps_points = parse_gps_points(txt_file_path)
    kml_data = create_kml(gps_points)
    create_kmz(kml_data, kmz_file_path)
    print("finish")

if __name__ == '__main__':
    main()
