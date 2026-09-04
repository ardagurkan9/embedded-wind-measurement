<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="26008000">
	<Property Name="NI.LV.All.SaveVersion" Type="Str">26.0</Property>
	<Property Name="NI.LV.All.SourceOnly" Type="Bool">true</Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Application" Type="Folder">
			<Item Name="Real Wind Monitor.vi" Type="VI" URL="../Real Wind Monitor.vi"/>
		</Item>
		<Item Name="Data" Type="Folder">
			<Item Name="Real Wind Sample.ctl" Type="VI" URL="../Real Wind Sample.ctl"/>
		</Item>
		<Item Name="Logging" Type="Folder">
			<Item Name="Write Real Wind Sample.vi" Type="VI" URL="../Write Real Wind Sample.vi"/>
		</Item>
		<Item Name="Protocol" Type="Folder">
			<Item Name="Calculate Modbus CRC16.vi..vi" Type="VI" URL="../Calculate Modbus CRC16.vi..vi"/>
			<Item Name="Decode FST200 Direction Response.vi" Type="VI" URL="../Decode FST200 Direction Response.vi"/>
			<Item Name="Decode FST200 Speed Response.vi" Type="VI" URL="../Decode FST200 Speed Response.vi"/>
			<Item Name="Parse Master Frame.vi" Type="VI" URL="../Parse Master Frame.vi"/>
		</Item>
		<Item Name="Dependencies" Type="Dependencies"/>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
