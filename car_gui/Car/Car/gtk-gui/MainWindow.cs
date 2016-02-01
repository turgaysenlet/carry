
// This file has been generated by the GUI designer. Do not modify.
public partial class MainWindow
{
	private global::Gtk.Table table1;
	private global::Gtk.Button buttonCloseAll;
	private global::Gtk.Button buttonExit;
	private global::Gtk.ToggleButton buttonGps;
	private global::Gtk.ToggleButton buttonImage;
	private global::Gtk.ToggleButton buttonRecord;
	private global::Gtk.ToggleButton buttonRoscore;
	private global::Gtk.ToggleButton buttonUsb;
	private global::Gtk.Button buttonUvc;

	protected virtual void Build ()
	{
		global::Stetic.Gui.Initialize (this);
		// Widget MainWindow
		this.Name = "MainWindow";
		this.Title = global::Mono.Unix.Catalog.GetString ("MainWindow");
		this.WindowPosition = ((global::Gtk.WindowPosition)(4));
		this.BorderWidth = ((uint)(6));
		// Container child MainWindow.Gtk.Container+ContainerChild
		this.table1 = new global::Gtk.Table (((uint)(3)), ((uint)(3)), false);
		this.table1.Name = "table1";
		this.table1.RowSpacing = ((uint)(6));
		this.table1.ColumnSpacing = ((uint)(6));
		// Container child table1.Gtk.Table+TableChild
		this.buttonCloseAll = new global::Gtk.Button ();
		this.buttonCloseAll.WidthRequest = 300;
		this.buttonCloseAll.HeightRequest = 300;
		this.buttonCloseAll.CanFocus = true;
		this.buttonCloseAll.Name = "buttonCloseAll";
		this.buttonCloseAll.UseUnderline = true;
		// Container child buttonCloseAll.Gtk.Container+ContainerChild
		global::Gtk.Alignment w1 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w2 = new global::Gtk.HBox ();
		w2.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w3 = new global::Gtk.Image ();
		w3.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "stock_dialog-warning", global::Gtk.IconSize.Dialog);
		w2.Add (w3);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w5 = new global::Gtk.Label ();
		w5.LabelProp = global::Mono.Unix.Catalog.GetString ("Close All");
		w5.UseUnderline = true;
		w2.Add (w5);
		w1.Add (w2);
		this.buttonCloseAll.Add (w1);
		this.table1.Add (this.buttonCloseAll);
		global::Gtk.Table.TableChild w9 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonCloseAll]));
		w9.TopAttach = ((uint)(1));
		w9.BottomAttach = ((uint)(2));
		w9.LeftAttach = ((uint)(2));
		w9.RightAttach = ((uint)(3));
		w9.XOptions = ((global::Gtk.AttachOptions)(4));
		w9.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonExit = new global::Gtk.Button ();
		this.buttonExit.WidthRequest = 653;
		this.buttonExit.HeightRequest = 300;
		this.buttonExit.CanFocus = true;
		this.buttonExit.Name = "buttonExit";
		this.buttonExit.UseUnderline = true;
		// Container child buttonExit.Gtk.Container+ContainerChild
		global::Gtk.Alignment w10 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w11 = new global::Gtk.HBox ();
		w11.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w12 = new global::Gtk.Image ();
		w12.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "stock_delete", global::Gtk.IconSize.Dialog);
		w11.Add (w12);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w14 = new global::Gtk.Label ();
		w14.LabelProp = global::Mono.Unix.Catalog.GetString ("Exit");
		w14.UseUnderline = true;
		w11.Add (w14);
		w10.Add (w11);
		this.buttonExit.Add (w10);
		this.table1.Add (this.buttonExit);
		global::Gtk.Table.TableChild w18 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonExit]));
		w18.TopAttach = ((uint)(2));
		w18.BottomAttach = ((uint)(3));
		w18.LeftAttach = ((uint)(1));
		w18.RightAttach = ((uint)(3));
		w18.YOptions = ((global::Gtk.AttachOptions)(1));
		// Container child table1.Gtk.Table+TableChild
		this.buttonGps = new global::Gtk.ToggleButton ();
		this.buttonGps.WidthRequest = 300;
		this.buttonGps.HeightRequest = 300;
		this.buttonGps.CanFocus = true;
		this.buttonGps.Name = "buttonGps";
		this.buttonGps.UseUnderline = true;
		// Container child buttonGps.Gtk.Container+ContainerChild
		global::Gtk.Alignment w19 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w20 = new global::Gtk.HBox ();
		w20.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w21 = new global::Gtk.Image ();
		w21.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "stock_internet", global::Gtk.IconSize.Dialog);
		w20.Add (w21);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w23 = new global::Gtk.Label ();
		w23.LabelProp = global::Mono.Unix.Catalog.GetString ("GPS");
		w23.UseUnderline = true;
		w20.Add (w23);
		w19.Add (w20);
		this.buttonGps.Add (w19);
		this.table1.Add (this.buttonGps);
		global::Gtk.Table.TableChild w27 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonGps]));
		w27.TopAttach = ((uint)(2));
		w27.BottomAttach = ((uint)(3));
		w27.XOptions = ((global::Gtk.AttachOptions)(4));
		w27.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonImage = new global::Gtk.ToggleButton ();
		this.buttonImage.WidthRequest = 300;
		this.buttonImage.HeightRequest = 300;
		this.buttonImage.CanFocus = true;
		this.buttonImage.Name = "buttonImage";
		this.buttonImage.UseUnderline = true;
		// Container child buttonImage.Gtk.Container+ContainerChild
		global::Gtk.Alignment w28 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w29 = new global::Gtk.HBox ();
		w29.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w30 = new global::Gtk.Image ();
		w30.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "stock_zoom-page", global::Gtk.IconSize.Dialog);
		w29.Add (w30);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w32 = new global::Gtk.Label ();
		w32.LabelProp = global::Mono.Unix.Catalog.GetString ("Show Image");
		w32.UseUnderline = true;
		w29.Add (w32);
		w28.Add (w29);
		this.buttonImage.Add (w28);
		this.table1.Add (this.buttonImage);
		global::Gtk.Table.TableChild w36 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonImage]));
		w36.LeftAttach = ((uint)(2));
		w36.RightAttach = ((uint)(3));
		w36.XOptions = ((global::Gtk.AttachOptions)(4));
		w36.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonRecord = new global::Gtk.ToggleButton ();
		this.buttonRecord.WidthRequest = 300;
		this.buttonRecord.HeightRequest = 300;
		this.buttonRecord.CanFocus = true;
		this.buttonRecord.Name = "buttonRecord";
		this.buttonRecord.UseUnderline = true;
		// Container child buttonRecord.Gtk.Container+ContainerChild
		global::Gtk.Alignment w37 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w38 = new global::Gtk.HBox ();
		w38.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w39 = new global::Gtk.Image ();
		w39.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "stock_media-rec", global::Gtk.IconSize.Dialog);
		w38.Add (w39);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w41 = new global::Gtk.Label ();
		w41.LabelProp = global::Mono.Unix.Catalog.GetString ("Record");
		w41.UseUnderline = true;
		w38.Add (w41);
		w37.Add (w38);
		this.buttonRecord.Add (w37);
		this.table1.Add (this.buttonRecord);
		global::Gtk.Table.TableChild w45 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonRecord]));
		w45.TopAttach = ((uint)(1));
		w45.BottomAttach = ((uint)(2));
		w45.LeftAttach = ((uint)(1));
		w45.RightAttach = ((uint)(2));
		w45.XOptions = ((global::Gtk.AttachOptions)(4));
		w45.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonRoscore = new global::Gtk.ToggleButton ();
		this.buttonRoscore.WidthRequest = 300;
		this.buttonRoscore.HeightRequest = 300;
		this.buttonRoscore.CanFocus = true;
		this.buttonRoscore.Name = "buttonRoscore";
		this.buttonRoscore.UseUnderline = true;
		// Container child buttonRoscore.Gtk.Container+ContainerChild
		global::Gtk.Alignment w46 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w47 = new global::Gtk.HBox ();
		w47.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w48 = new global::Gtk.Image ();
		w48.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "gtk-dialog-authentication", global::Gtk.IconSize.Dialog);
		w47.Add (w48);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w50 = new global::Gtk.Label ();
		w50.LabelProp = global::Mono.Unix.Catalog.GetString ("Roscore");
		w50.UseUnderline = true;
		w47.Add (w50);
		w46.Add (w47);
		this.buttonRoscore.Add (w46);
		this.table1.Add (this.buttonRoscore);
		global::Gtk.Table.TableChild w54 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonRoscore]));
		w54.XOptions = ((global::Gtk.AttachOptions)(4));
		w54.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonUsb = new global::Gtk.ToggleButton ();
		this.buttonUsb.WidthRequest = 300;
		this.buttonUsb.HeightRequest = 300;
		this.buttonUsb.CanFocus = true;
		this.buttonUsb.Name = "buttonUsb";
		this.buttonUsb.UseUnderline = true;
		// Container child buttonUsb.Gtk.Container+ContainerChild
		global::Gtk.Alignment w55 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w56 = new global::Gtk.HBox ();
		w56.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w57 = new global::Gtk.Image ();
		w57.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "gtk-missing-image", global::Gtk.IconSize.Dialog);
		w56.Add (w57);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w59 = new global::Gtk.Label ();
		w59.LabelProp = global::Mono.Unix.Catalog.GetString ("USB Camera");
		w59.UseUnderline = true;
		w56.Add (w59);
		w55.Add (w56);
		this.buttonUsb.Add (w55);
		this.table1.Add (this.buttonUsb);
		global::Gtk.Table.TableChild w63 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonUsb]));
		w63.LeftAttach = ((uint)(1));
		w63.RightAttach = ((uint)(2));
		w63.XOptions = ((global::Gtk.AttachOptions)(4));
		w63.YOptions = ((global::Gtk.AttachOptions)(4));
		// Container child table1.Gtk.Table+TableChild
		this.buttonUvc = new global::Gtk.Button ();
		this.buttonUvc.WidthRequest = 300;
		this.buttonUvc.HeightRequest = 300;
		this.buttonUvc.CanFocus = true;
		this.buttonUvc.Name = "buttonUvc";
		this.buttonUvc.UseUnderline = true;
		// Container child buttonUvc.Gtk.Container+ContainerChild
		global::Gtk.Alignment w64 = new global::Gtk.Alignment (0.5F, 0.5F, 0F, 0F);
		// Container child GtkAlignment.Gtk.Container+ContainerChild
		global::Gtk.HBox w65 = new global::Gtk.HBox ();
		w65.Spacing = 2;
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Image w66 = new global::Gtk.Image ();
		w66.Pixbuf = global::Stetic.IconLoader.LoadIcon (this, "gtk-select-color", global::Gtk.IconSize.Dialog);
		w65.Add (w66);
		// Container child GtkHBox.Gtk.Container+ContainerChild
		global::Gtk.Label w68 = new global::Gtk.Label ();
		w68.LabelProp = global::Mono.Unix.Catalog.GetString ("Set Camera Parameters");
		w68.UseUnderline = true;
		w65.Add (w68);
		w64.Add (w65);
		this.buttonUvc.Add (w64);
		this.table1.Add (this.buttonUvc);
		global::Gtk.Table.TableChild w72 = ((global::Gtk.Table.TableChild)(this.table1 [this.buttonUvc]));
		w72.TopAttach = ((uint)(1));
		w72.BottomAttach = ((uint)(2));
		w72.XOptions = ((global::Gtk.AttachOptions)(4));
		w72.YOptions = ((global::Gtk.AttachOptions)(4));
		this.Add (this.table1);
		if ((this.Child != null)) {
			this.Child.ShowAll ();
		}
		this.DefaultWidth = 971;
		this.DefaultHeight = 924;
		this.Show ();
		this.DeleteEvent += new global::Gtk.DeleteEventHandler (this.OnDeleteEvent);
		this.buttonUvc.Clicked += new global::System.EventHandler (this.buttonUvcClicked);
		this.buttonUsb.Clicked += new global::System.EventHandler (this.buttonUsbClicked);
		this.buttonRoscore.Clicked += new global::System.EventHandler (this.buttonRoscoreClicked);
		this.buttonRecord.Clicked += new global::System.EventHandler (this.buttonRecordClicked);
		this.buttonImage.Clicked += new global::System.EventHandler (this.buttonImageClicked);
		this.buttonGps.Clicked += new global::System.EventHandler (this.buttonGpsClicked);
		this.buttonExit.Clicked += new global::System.EventHandler (this.buttonExitClicked);
		this.buttonCloseAll.Clicked += new global::System.EventHandler (this.buttonCloseAllClicked);
	}
}