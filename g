<!-- Add this namespace on your Window/UserControl root -->
<!-- xmlns:ghelix="http://helixToolkit.org/wpf/SharpDX" -->

<Grid>
    <ghelix:Viewport3DX x:Name="Viewport"
                        ZoomExtentsWhenLoaded="True"
                        CameraRotationMode="Trackball"
                        UseDefaultGestures="True"
                        BackgroundColor="Black">

        <!-- Lights (SharpDX side uses Light3D types) -->
        <ghelix:AmbientLight3D Color="DimGray"/>
        <ghelix:DirectionalLight3D Color="White" Direction="-1,-1,-1"/>
        <ghelix:DirectionalLight3D Color="White" Direction=" 1,-1,-0.1"/>

        <!-- Root container -->
        <ghelix:GroupModel3D x:Name="SceneRoot">

            <!-- 3D mesh rendered here -->
            <ghelix:MeshGeometryModel3D x:Name="MeshModel"
                                        CullMode="Back"
                                        IsHitTestVisible="True"/>
            
            <!-- 3D point cloud rendered here -->
            <ghelix:PointGeometryModel3D x:Name="PointCloudPoints"
                                         Size="1.5"
                                         IsHitTestVisible="True"/>

            <!-- projected points onto a plane -->
            <ghelix:PointGeometryModel3D x:Name="ProjectedPoints"
                                         Size="1.5"
                                         IsHitTestVisible="True"/>

            <!-- container for fitted planes -->
            <ghelix:GroupModel3D x:Name="PlanesModel" />

            <!-- container for fitted slices -->
            <ghelix:GroupModel3D x:Name="SliceModel" />

        </ghelix:GroupModel3D>
    </ghelix:Viewport3DX>
</Grid>