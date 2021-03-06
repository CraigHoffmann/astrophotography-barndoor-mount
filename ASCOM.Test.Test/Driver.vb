'tabs=4
' --------------------------------------------------------------------------------
' TODO fill in this information for your driver, then remove this line!
'
' Simple Autoguide (ASCOM) interface
'
' Description:	ASCOM Telescope driver for Simple Autoguide Control Interface
'               Developed for use with a arduino controlled Barn Door mount
'
' Implements:	ASCOM Telescope interface (minimal) version: 1.0
' Author:		Craig Hoffmann
'
' Edit Log:
'
' Date			Who	Vers	Description
' -----------	---	-----	-------------------------------------------------------
' dd-mmm-yyyy	XXX	1.0.0	Initial edit, from Telescope template
' ---------------------------------------------------------------------------------
'
'
' Your driver's ID is ASCOM.Test.Telescope
'
' The Guid attribute sets the CLSID for ASCOM.DeviceName.Telescope
' The ClassInterface/None attribute prevents an empty interface called
' _Telescope from being created and used as the [default] interface
'

' This definition is used to select code that's only applicable for one device type
#Const Device = "Telescope"

Imports System
Imports System.Collections
Imports System.Collections.Generic
Imports System.Globalization
Imports System.Runtime.InteropServices
Imports System.Text
Imports ASCOM
Imports ASCOM.Astrometry
Imports ASCOM.Astrometry.AstroUtils
Imports ASCOM.DeviceInterface
Imports ASCOM.Utilities

<Guid("fe31e1b7-1339-4cec-afdf-cf8602e4f205")>
<ClassInterface(ClassInterfaceType.None)>
Public Class Telescope

    ' The Guid attribute sets the CLSID for ASCOM.Test.Telescope
    ' The ClassInterface/None attribute prevents an empty interface called
    ' _Test from being created and used as the [default] interface

    ' TODO Replace the not implemented exceptions with code to implement the function or
    ' throw the appropriate ASCOM exception.
    '
    Implements ITelescopeV3


    '
    ' Driver ID and descriptive string that shows in the Chooser
    '
    Friend Shared driverID As String = "ASCOM.Test.Telescope"
    Private Shared driverDescription As String = "Simple Autoguide Mount (ASCOM) Interface"
    Private objSerial As ASCOM.Utilities.Serial

    Friend Shared comPortProfileName As String = "COM Port" 'Constants used for Profile persistence
    Friend Shared traceStateProfileName As String = "Trace Level"
    Friend Shared comPortDefault As String = "COM1"
    Friend Shared traceStateDefault As String = "False"

    Friend Shared comPort As String ' Variables to hold the current device configuration
    Friend Shared traceState As Boolean

    Private connectedState As Boolean ' Private variable to hold the connected state
    Private utilities As Util ' Private variable to hold an ASCOM Utilities object
    Private astroUtilities As AstroUtils ' Private variable to hold an AstroUtils object to provide the Range method
    Private TL As TraceLogger ' Private variable to hold the trace logger object (creates a diagnostic log file with information that you specify)

    '
    ' Constructor - Must be public for COM registration!
    '
    Public Sub New()

        ReadProfile() ' Read device configuration from the ASCOM Profile store
        TL = New TraceLogger("", "Test")
        TL.Enabled = traceState
        TL.LogMessage("Telescope", "Starting initialisation")

        connectedState = False ' Initialise connected to false
        utilities = New Util() ' Initialise util object
        astroUtilities = New AstroUtils 'Initialise new astro utilities object

        'CRAIG: Implement your additional construction here
        'Dim s As String
        's = Trim(Replace(objSerial.ReceiveTerminated(vbCr),vbLf,""))       ' Controller sends OK after start up/reset

        TL.LogMessage("Telescope", "Completed initialisation")
    End Sub

    '
    ' PUBLIC COM INTERFACE ITelescopeV3 IMPLEMENTATION
    '

#Region "Common properties and methods"
    ''' <summary>
    ''' Displays the Setup Dialog form.
    ''' If the user clicks the OK button to dismiss the form, then
    ''' the new settings are saved, otherwise the old values are reloaded.
    ''' THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
    ''' </summary>
    Public Sub SetupDialog() Implements ITelescopeV3.SetupDialog
        ' consider only showing the setup dialog if not connected
        ' or call a different dialog if connected
        If IsConnected Then
            System.Windows.Forms.MessageBox.Show("Already connected, just press OK")
        End If

        Using F As SetupDialogForm = New SetupDialogForm()
            Dim result As System.Windows.Forms.DialogResult = F.ShowDialog()
            If result = DialogResult.OK Then
                WriteProfile() ' Persist device configuration values to the ASCOM Profile store
            End If
        End Using
    End Sub

    Public ReadOnly Property SupportedActions() As ArrayList Implements ITelescopeV3.SupportedActions
        Get
            TL.LogMessage("SupportedActions Get", "Returning empty arraylist")
            Return New ArrayList()
        End Get
    End Property

    Public Function Action(ByVal ActionName As String, ByVal ActionParameters As String) As String Implements ITelescopeV3.Action
        Throw New ActionNotImplementedException("Action " & ActionName & " is not supported by this driver")
    End Function

    Public Sub CommandBlind(ByVal Command As String, Optional ByVal Raw As Boolean = False) Implements ITelescopeV3.CommandBlind
        CheckConnected("CommandBlind")
        ' TODO The optional CommandBlind method should either be implemented OR throw a MethodNotImplementedException
        ' If implemented, CommandBlind must send the supplied command to the mount And return immediately without waiting for a response

        Throw New MethodNotImplementedException("CommandBlind")
    End Sub

    Public Function CommandBool(ByVal Command As String, Optional ByVal Raw As Boolean = False) As Boolean _
        Implements ITelescopeV3.CommandBool
        CheckConnected("CommandBool")
        ' TODO The optional CommandBool method should either be implemented OR throw a MethodNotImplementedException
        ' If implemented, CommandBool must send the supplied command to the mount, wait for a response and parse this to return a True Or False value

        ' Dim retString as String = CommandString(command, raw) ' Send the command And wait for the response
        ' Dim retBool as Boolean = XXXXXXXXXXXXX ' Parse the returned string And create a boolean True / False value
        ' Return retBool ' Return the boolean value to the client

        Throw New MethodNotImplementedException("CommandBool")
    End Function

    Public Function CommandString(ByVal Command As String, Optional ByVal Raw As Boolean = False) As String _
        Implements ITelescopeV3.CommandString
        CheckConnected("CommandString")
        ' TODO The optional CommandString method should either be implemented OR throw a MethodNotImplementedException
        ' If implemented, CommandString must send the supplied command to the mount and wait for a response before returning this to the client

        Throw New MethodNotImplementedException("CommandString")
    End Function

    Public Property Connected() As Boolean Implements ITelescopeV3.Connected
        Get
            TL.LogMessage("Connected Get", IsConnected.ToString())
            Return IsConnected
        End Get
        Set(value As Boolean)
            TL.LogMessage("Connected Set", value.ToString())
            If value = IsConnected Then
                Return
            End If

            If value Then
                connectedState = True
                TL.LogMessage("Connected Set", "Connecting to port " + comPort)
                ' CRAIG connect to the device
                objSerial = New ASCOM.Utilities.Serial
                objSerial.Port = Val(Right(comPort, Len(comPort) - 3))
                objSerial.Speed = 115200
                objSerial.ClearBuffers()
                objSerial.Connected = True
            End If
        End Set
    End Property

    Public ReadOnly Property Description As String Implements ITelescopeV3.Description
        Get
            ' this pattern seems to be needed to allow a public property to return a private field
            Dim d As String = driverDescription
            TL.LogMessage("Description Get", d)
            Return d
        End Get
    End Property

    Public ReadOnly Property DriverInfo As String Implements ITelescopeV3.DriverInfo
        Get
            Dim m_version As Version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version
            ' TODO customise this driver description
            Dim s_driverInfo As String = "Information about the driver itself. Version: " + m_version.Major.ToString() + "." + m_version.Minor.ToString()
            TL.LogMessage("DriverInfo Get", s_driverInfo)
            Return s_driverInfo
        End Get
    End Property

    Public ReadOnly Property DriverVersion() As String Implements ITelescopeV3.DriverVersion
        Get
            ' Get our own assembly and report its version number
            TL.LogMessage("DriverVersion Get", Reflection.Assembly.GetExecutingAssembly.GetName.Version.ToString(2))
            Return Reflection.Assembly.GetExecutingAssembly.GetName.Version.ToString(2)
        End Get
    End Property

    Public ReadOnly Property InterfaceVersion() As Short Implements ITelescopeV3.InterfaceVersion
        Get
            TL.LogMessage("InterfaceVersion Get", "3")
            Return 3
        End Get
    End Property

    Public ReadOnly Property Name As String Implements ITelescopeV3.Name
        Get
            Dim s_name As String = "Simple AutoGuide Mount (ASCOM) Interface"     ' CRAIG
            TL.LogMessage("Name Get", s_name)
            Return s_name
        End Get
    End Property

    Public Sub Dispose() Implements ITelescopeV3.Dispose
        ' Clean up the trace logger and util objects
        TL.Enabled = False
        TL.Dispose()
        TL = Nothing
        utilities.Dispose()
        utilities = Nothing
        astroUtilities.Dispose()
        astroUtilities = Nothing
    End Sub

#End Region

#Region "ITelescope Implementation"
    Public Sub AbortSlew() Implements ITelescopeV3.AbortSlew
        TL.LogMessage("AbortSlew", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("AbortSlew")
    End Sub

    Public ReadOnly Property AlignmentMode() As AlignmentModes Implements ITelescopeV3.AlignmentMode
        Get
            TL.LogMessage("AlignmentMode Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("AlignmentMode", False)
        End Get
    End Property

    Public ReadOnly Property Altitude() As Double Implements ITelescopeV3.Altitude
        Get
            TL.LogMessage("Altitude", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("Altitude", False)
        End Get
    End Property

    Public ReadOnly Property ApertureArea() As Double Implements ITelescopeV3.ApertureArea
        Get
            TL.LogMessage("ApertureArea Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("ApertureArea", False)
        End Get
    End Property

    Public ReadOnly Property ApertureDiameter() As Double Implements ITelescopeV3.ApertureDiameter
        Get
            TL.LogMessage("ApertureDiameter Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("ApertureDiameter", False)
        End Get
    End Property

    Public ReadOnly Property AtHome() As Boolean Implements ITelescopeV3.AtHome
        Get
            TL.LogMessage("AtHome", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property AtPark() As Boolean Implements ITelescopeV3.AtPark
        Get
            TL.LogMessage("AtPark", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public Function AxisRates(Axis As TelescopeAxes) As IAxisRates Implements ITelescopeV3.AxisRates
        TL.LogMessage("AxisRates", "Get - " & Axis.ToString())
        Return New AxisRates(Axis)
    End Function

    Public ReadOnly Property Azimuth() As Double Implements ITelescopeV3.Azimuth
        Get
            TL.LogMessage("Azimuth Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("Azimuth", False)
        End Get
    End Property

    Public ReadOnly Property CanFindHome() As Boolean Implements ITelescopeV3.CanFindHome
        Get
            TL.LogMessage("CanFindHome", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public Function CanMoveAxis(Axis As TelescopeAxes) As Boolean Implements ITelescopeV3.CanMoveAxis
        TL.LogMessage("CanMoveAxis", "Get - " & Axis.ToString())
        Select Case Axis
            Case TelescopeAxes.axisPrimary
                Return False
            Case TelescopeAxes.axisSecondary
                Return False
            Case TelescopeAxes.axisTertiary
                Return False
            Case Else
                Throw New InvalidValueException("CanMoveAxis", Axis.ToString(), "0 to 2")
        End Select
    End Function

    Public ReadOnly Property CanPark() As Boolean Implements ITelescopeV3.CanPark
        Get
            TL.LogMessage("CanPark", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanPulseGuide() As Boolean Implements ITelescopeV3.CanPulseGuide
        Get
            ' CRAIG - enable pulse guide support
            TL.LogMessage("CanPulseGuide", "Get - " & True.ToString())
            Return True
        End Get
    End Property

    Public ReadOnly Property CanSetDeclinationRate() As Boolean Implements ITelescopeV3.CanSetDeclinationRate
        Get
            TL.LogMessage("CanSetDeclinationRate", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSetGuideRates() As Boolean Implements ITelescopeV3.CanSetGuideRates
        Get
            TL.LogMessage("CanSetGuideRates", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSetPark() As Boolean Implements ITelescopeV3.CanSetPark
        Get
            TL.LogMessage("CanSetPark", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSetPierSide() As Boolean Implements ITelescopeV3.CanSetPierSide
        Get
            TL.LogMessage("CanSetPierSide", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSetRightAscensionRate() As Boolean Implements ITelescopeV3.CanSetRightAscensionRate
        Get
            TL.LogMessage("CanSetRightAscensionRate", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSetTracking() As Boolean Implements ITelescopeV3.CanSetTracking
        Get
            TL.LogMessage("CanSetTracking", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSlew() As Boolean Implements ITelescopeV3.CanSlew
        Get
            TL.LogMessage("CanSlew", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSlewAltAz() As Boolean Implements ITelescopeV3.CanSlewAltAz
        Get
            TL.LogMessage("CanSlewAltAz", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSlewAltAzAsync() As Boolean Implements ITelescopeV3.CanSlewAltAzAsync
        Get
            TL.LogMessage("CanSlewAltAzAsync", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSlewAsync() As Boolean Implements ITelescopeV3.CanSlewAsync
        Get
            TL.LogMessage("CanSlewAsync", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSync() As Boolean Implements ITelescopeV3.CanSync
        Get
            TL.LogMessage("CanSync", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanSyncAltAz() As Boolean Implements ITelescopeV3.CanSyncAltAz
        Get
            TL.LogMessage("CanSyncAltAz", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property CanUnpark() As Boolean Implements ITelescopeV3.CanUnpark
        Get
            TL.LogMessage("CanUnpark", "Get - " & False.ToString())
            Return False
        End Get
    End Property

    Public ReadOnly Property Declination() As Double Implements ITelescopeV3.Declination
        Get
            Dim declination__1 As Double = 0.0
            TL.LogMessage("Declination", "Get - " & utilities.DegreesToDMS(declination__1, ":", ":"))
            Return declination__1
        End Get
    End Property

    Public Property DeclinationRate() As Double Implements ITelescopeV3.DeclinationRate
        Get
            Dim declination As Double = 0.0
            TL.LogMessage("DeclinationRate", "Get - " & declination.ToString())
            Return declination
        End Get
        Set(value As Double)
            TL.LogMessage("DeclinationRate Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("DeclinationRate", True)
        End Set
    End Property

    Public Function DestinationSideOfPier(RightAscension As Double, Declination As Double) As PierSide Implements ITelescopeV3.DestinationSideOfPier
        TL.LogMessage("DestinationSideOfPier Get", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("DestinationSideOfPier")
    End Function

    Public Property DoesRefraction() As Boolean Implements ITelescopeV3.DoesRefraction
        Get
            TL.LogMessage("DoesRefraction Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("DoesRefraction", False)
        End Get
        Set(value As Boolean)
            TL.LogMessage("DoesRefraction Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("DoesRefraction", True)
        End Set
    End Property

    Public ReadOnly Property EquatorialSystem() As EquatorialCoordinateType Implements ITelescopeV3.EquatorialSystem
        Get
            Dim equatorialSystem__1 As EquatorialCoordinateType = EquatorialCoordinateType.equTopocentric
            TL.LogMessage("DeclinationRate", "Get - " & equatorialSystem__1.ToString())
            Return equatorialSystem__1
        End Get
    End Property

    Public Sub FindHome() Implements ITelescopeV3.FindHome
        TL.LogMessage("FindHome", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("FindHome")
    End Sub

    Public ReadOnly Property FocalLength() As Double Implements ITelescopeV3.FocalLength
        Get
            TL.LogMessage("FocalLength Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("FocalLength", False)
        End Get
    End Property

    Public Property GuideRateDeclination() As Double Implements ITelescopeV3.GuideRateDeclination
        Get
            ' CRAIG - Get the guide rate
            'objSerial.Transmit("GR DEC" & vbCr)
            Dim s As String
            Dim rate As Double
            's = Trim(Replace(objSerial.ReceiveTerminated(vbCr), vbLf, ""))
            s = GetResponse("GR DEC" & vbCr)
            rate = Val(s)
            If ((rate < 0.1) Or (rate > 0.9)) Then
                rate = 0.5
                TL.LogMessage("GuideRateRightDeclination Get", "Invalid value received: " & s)
            Else
                TL.LogMessage("GuideRateRightDeclination Get", "Received: " & s)
            End If
            Return rate
        End Get
        Set(value As Double)
            TL.LogMessage("GuideRateDeclination Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("GuideRateDeclination", True)
        End Set
    End Property

    Public Property GuideRateRightAscension() As Double Implements ITelescopeV3.GuideRateRightAscension
        Get
            ' CRAIG - Get the guide rate
            'objSerial.Transmit("GR RA" & vbCr)
            Dim s As String
            Dim rate As Double
            's = Trim(Replace(objSerial.ReceiveTerminated(vbCr), vbLf, ""))
            s = GetResponse("GR RA" & vbCr)
            rate = Val(s)
            If ((rate < 0.1) Or (rate > 0.9)) Then
                rate = 0.5
                TL.LogMessage("GuideRateRightAscension Get", "Invalid value received: " & s)
            Else
                TL.LogMessage("GuideRateRightAscension Get", "Received: " & s)
            End If
            Return rate
        End Get
        Set(value As Double)
            TL.LogMessage("GuideRateRightAscension Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("GuideRateRightAscension", True)
        End Set
    End Property

    Public ReadOnly Property IsPulseGuiding() As Boolean Implements ITelescopeV3.IsPulseGuiding
        Get
            ' CRAIG - check if the controller is in middle of guide pulse
            'objSerial.Transmit("GQ" & vbCr)
            Dim s As String

            's = Trim(Replace(objSerial.ReceiveTerminated(vbCr), vbLf, ""))
            s = GetResponse("GQ" & vbCr)
            If (Left(s, 1) = "0") Then
                TL.LogMessage("IsPulseGuiding Get", "False: " & s)
                Return False
            Else
                TL.LogMessage("IsPulseGuiding Get", "True: " & s)
                Return True
            End If
        End Get
    End Property

    Public Sub MoveAxis(Axis As TelescopeAxes, Rate As Double) Implements ITelescopeV3.MoveAxis
        TL.LogMessage("MoveAxis", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("MoveAxis")
    End Sub

    Public Sub Park() Implements ITelescopeV3.Park
        TL.LogMessage("Park", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("Park")
    End Sub

    Public Sub PulseGuide(Direction As GuideDirections, Duration As Integer) Implements ITelescopeV3.PulseGuide
        ' CRAIG - send a guide direction and duration (in mS) command to the controller
        Dim s As String
        Select Case Direction
            Case GuideDirections.guideEast
                'objSerial.Transmit("GE " & Duration.ToString("d") & vbCr)
                s = GetResponse("GE " & Duration.ToString("d") & vbCr)
            Case GuideDirections.guideWest
                'objSerial.Transmit("GW " & Duration.ToString("d") & vbCr)
                s = GetResponse("GW " & Duration.ToString("d") & vbCr)
            Case GuideDirections.guideNorth
                'objSerial.Transmit("GN " & Duration.ToString("d") & vbCr)
                s = GetResponse("GN " & Duration.ToString("d") & vbCr)
            Case GuideDirections.guideSouth
                'objSerial.Transmit("GS " & Duration.ToString("d") & vbCr)
                s = GetResponse("GS " & Duration.ToString("d") & vbCr)
        End Select
        TL.LogMessage("PulseGuide", "Sent duration " & Duration.ToString("d"))

        's = Trim(Replace(objSerial.ReceiveTerminated(vbCr), vbLf, ""))
        If (Left(s, 2) <> "OK") Then
            TL.LogMessage("PulseGuide", "Receive Error: " & s)
        End If
    End Sub

    Public ReadOnly Property RightAscension() As Double Implements ITelescopeV3.RightAscension
        Get
            Dim rightAscension__1 As Double = 0.0
            TL.LogMessage("RightAscension", "Get - " & utilities.HoursToHMS(rightAscension__1))
            Return rightAscension__1
        End Get
    End Property

    Public Property RightAscensionRate() As Double Implements ITelescopeV3.RightAscensionRate
        Get
            Dim rightAscensionRate__1 As Double = 0.0
            TL.LogMessage("RightAscensionRate", "Get - " & rightAscensionRate__1.ToString())
            Return rightAscensionRate__1
        End Get
        Set(value As Double)
            TL.LogMessage("RightAscensionRate Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("RightAscensionRate", True)
        End Set
    End Property

    Public Sub SetPark() Implements ITelescopeV3.SetPark
        TL.LogMessage("SetPark", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SetPark")
    End Sub

    Public Property SideOfPier() As PierSide Implements ITelescopeV3.SideOfPier
        Get
            TL.LogMessage("SideOfPier Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SideOfPier", False)
        End Get
        Set(value As PierSide)
            TL.LogMessage("SideOfPier Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SideOfPier", True)
        End Set
    End Property

    Public ReadOnly Property SiderealTime() As Double Implements ITelescopeV3.SiderealTime
        Get
            ' now using novas 3.1
            Dim lst As Double = 0.0
            Using novas As New ASCOM.Astrometry.NOVAS.NOVAS31
                Dim jd As Double = utilities.DateUTCToJulian(DateTime.UtcNow)
                novas.SiderealTime(jd, 0, novas.DeltaT(jd),
                                   Astrometry.GstType.GreenwichMeanSiderealTime,
                                   Astrometry.Method.EquinoxBased,
                                   Astrometry.Accuracy.Reduced,
                                   lst)
            End Using

            ' Allow for the longitude
            lst += SiteLongitude / 360.0 * 24.0

            ' Reduce to the range 0 to 24 hours
            lst = astroUtilities.ConditionRA(lst)

            TL.LogMessage("SiderealTime", "Get - " & lst.ToString())
            Return lst
        End Get
    End Property

    Public Property SiteElevation() As Double Implements ITelescopeV3.SiteElevation
        Get
            TL.LogMessage("SiteElevation Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteElevation", False)
        End Get
        Set(value As Double)
            TL.LogMessage("SiteElevation Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteElevation", True)
        End Set
    End Property

    Public Property SiteLatitude() As Double Implements ITelescopeV3.SiteLatitude
        Get
            TL.LogMessage("SiteLatitude Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteLatitude", False)
        End Get
        Set(value As Double)
            TL.LogMessage("SiteLatitude Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteLatitude", True)
        End Set
    End Property

    Public Property SiteLongitude() As Double Implements ITelescopeV3.SiteLongitude
        Get
            TL.LogMessage("SiteLongitude Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteLongitude", False)
        End Get
        Set(value As Double)
            TL.LogMessage("SiteLongitude Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SiteLongitude", True)
        End Set
    End Property

    Public Property SlewSettleTime() As Short Implements ITelescopeV3.SlewSettleTime
        Get
            TL.LogMessage("SlewSettleTime Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SlewSettleTime", False)
        End Get
        Set(value As Short)
            TL.LogMessage("SlewSettleTime Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("SlewSettleTime", True)
        End Set
    End Property

    Public Sub SlewToAltAz(Azimuth As Double, Altitude As Double) Implements ITelescopeV3.SlewToAltAz
        TL.LogMessage("SlewToAltAz", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToAltAz")
    End Sub

    Public Sub SlewToAltAzAsync(Azimuth As Double, Altitude As Double) Implements ITelescopeV3.SlewToAltAzAsync
        TL.LogMessage("SlewToAltAzAsync", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToAltAzAsync")
    End Sub

    Public Sub SlewToCoordinates(RightAscension As Double, Declination As Double) Implements ITelescopeV3.SlewToCoordinates
        TL.LogMessage("SlewToCoordinates", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToCoordinates")
    End Sub

    Public Sub SlewToCoordinatesAsync(RightAscension As Double, Declination As Double) Implements ITelescopeV3.SlewToCoordinatesAsync
        TL.LogMessage("SlewToCoordinatesAsync", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToCoordinatesAsync")
    End Sub

    Public Sub SlewToTarget() Implements ITelescopeV3.SlewToTarget
        TL.LogMessage("SlewToTarget", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToTarget")
    End Sub

    Public Sub SlewToTargetAsync() Implements ITelescopeV3.SlewToTargetAsync
        TL.LogMessage("    Public Sub SlewToTargetAsync() Implements ITelescopeV3.SlewToTargetAsync", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SlewToTargetAsync")
    End Sub

    Public ReadOnly Property Slewing() As Boolean Implements ITelescopeV3.Slewing
        Get
            TL.LogMessage("Slewing Get", "False - Mount not capable")
            Return False
            Throw New ASCOM.PropertyNotImplementedException("Slewing", False)
        End Get
    End Property

    Public Sub SyncToAltAz(Azimuth As Double, Altitude As Double) Implements ITelescopeV3.SyncToAltAz
        TL.LogMessage("SyncToAltAz", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SyncToAltAz")
    End Sub

    Public Sub SyncToCoordinates(RightAscension As Double, Declination As Double) Implements ITelescopeV3.SyncToCoordinates
        TL.LogMessage("SyncToCoordinates", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SyncToCoordinates")
    End Sub

    Public Sub SyncToTarget() Implements ITelescopeV3.SyncToTarget
        TL.LogMessage("SyncToTarget", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("SyncToTarget")
    End Sub

    Public Property TargetDeclination() As Double Implements ITelescopeV3.TargetDeclination
        Get
            TL.LogMessage("TargetDeclination Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TargetDeclination", False)
        End Get
        Set(value As Double)
            TL.LogMessage("TargetDeclination Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TargetDeclination", True)
        End Set
    End Property

    Public Property TargetRightAscension() As Double Implements ITelescopeV3.TargetRightAscension
        Get
            TL.LogMessage("TargetRightAscension Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TargetRightAscension", False)
        End Get
        Set(value As Double)
            TL.LogMessage("TargetRightAscension Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TargetRightAscension", True)
        End Set
    End Property

    Public Property Tracking() As Boolean Implements ITelescopeV3.Tracking
        Get
            Dim tracking__1 As Boolean = True
            TL.LogMessage("Tracking", "Get - " & tracking__1.ToString())
            Return tracking__1
        End Get
        Set(value As Boolean)
            TL.LogMessage("Tracking Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("Tracking", True)
        End Set
    End Property

    Public Property TrackingRate() As DriveRates Implements ITelescopeV3.TrackingRate
        Get
            TL.LogMessage("TrackingRate Get", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TrackingRate", False)
        End Get
        Set(value As DriveRates)
            TL.LogMessage("TrackingRate Set", "Not implemented")
            Throw New ASCOM.PropertyNotImplementedException("TrackingRate", True)
        End Set
    End Property

    Public ReadOnly Property TrackingRates() As ITrackingRates Implements ITelescopeV3.TrackingRates
        Get
            Dim trackingRates__1 As ITrackingRates = New TrackingRates()
            TL.LogMessage("TrackingRates", "Get - ")
            For Each driveRate As DriveRates In trackingRates__1
                TL.LogMessage("TrackingRates", "Get - " & driveRate.ToString())
            Next
            Return trackingRates__1
        End Get
    End Property

    Public Property UTCDate() As DateTime Implements ITelescopeV3.UTCDate
        Get
            Dim utcDate__1 As DateTime = DateTime.UtcNow
            TL.LogMessage("UTCDate", String.Format("Get - {0}", utcDate__1))
            Return utcDate__1
        End Get
        Set(value As DateTime)
            Throw New ASCOM.PropertyNotImplementedException("UTCDate", True)
        End Set
    End Property

    Public Sub Unpark() Implements ITelescopeV3.Unpark
        TL.LogMessage("Unpark", "Not implemented")
        Throw New ASCOM.MethodNotImplementedException("Unpark")
    End Sub

#End Region

#Region "Private properties and methods"
    ' here are some useful properties and methods that can be used as required
    ' to help with

#Region "ASCOM Registration"

    Private Shared Sub RegUnregASCOM(ByVal bRegister As Boolean)

        Using P As New Profile() With {.DeviceType = "Telescope"}
            If bRegister Then
                P.Register(driverID, driverDescription)
            Else
                P.Unregister(driverID)
            End If
        End Using

    End Sub

    <ComRegisterFunction()>
    Public Shared Sub RegisterASCOM(ByVal T As Type)

        RegUnregASCOM(True)

    End Sub

    <ComUnregisterFunction()>
    Public Shared Sub UnregisterASCOM(ByVal T As Type)

        RegUnregASCOM(False)

    End Sub

#End Region

    ''' <summary>
    ''' Returns true if there is a valid connection to the driver hardware
    ''' </summary>
    Private ReadOnly Property IsConnected As Boolean
        Get
            ' CRAIG check that the driver hardware connection exists and is connected to the hardware
            If objSerial Is Nothing Then
                Return False
            Else
                If objSerial.Connected Then
                    Return True
                Else
                    Return False
                End If
            End If
            'Return connectedState
        End Get
    End Property

    ''' <summary>
    ''' Use this function to throw an exception if we aren't connected to the hardware
    ''' </summary>
    ''' <param name="message"></param>
    Private Sub CheckConnected(ByVal message As String)
        If Not IsConnected Then
            Throw New NotConnectedException(message)
        End If
    End Sub

    ''' <summary>
    ''' Read the device configuration from the ASCOM Profile store
    ''' </summary>
    Friend Sub ReadProfile()
        Using driverProfile As New Profile()
            driverProfile.DeviceType = "Telescope"
            traceState = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, String.Empty, traceStateDefault))
            comPort = driverProfile.GetValue(driverID, comPortProfileName, String.Empty, comPortDefault)
        End Using
    End Sub

    ''' <summary>
    ''' Write the device configuration to the  ASCOM  Profile store
    ''' </summary>
    Friend Sub WriteProfile()
        Using driverProfile As New Profile()
            driverProfile.DeviceType = "Telescope"
            driverProfile.WriteValue(driverID, traceStateProfileName, traceState.ToString())
            driverProfile.WriteValue(driverID, comPortProfileName, comPort.ToString())
        End Using

    End Sub

    ' CRAIG - function to clean received strings from controller

    Private Function GetResponse(ByVal cmd_str As String) As String
        Dim clean_str As String
        objSerial.Transmit(cmd_str)
        clean_str = objSerial.ReceiveTerminated(vbCr)
        clean_str = Replace(clean_str, vbCr, "")
        clean_str = clean_str.Replace(vbLf, "")
        clean_str = clean_str.Trim()
        Return clean_str
    End Function

#End Region

End Class
