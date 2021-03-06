package com.stu.opencv;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity implements View.OnClickListener{

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    private SeekBar mHoughSB;
    private int mHoughValue = 150;
    private ImageView mImg;
    private Button mBtn1, mBtnGray;
    private Button mBtnPre, mBtnNext;
    private int[] mResId = {R.drawable.road_26,
            R.drawable.road_0, R.drawable.road_1, R.drawable.road_2, R.drawable.road_3, R.drawable.road_4,
            R.drawable.road_5, R.drawable.road_6, R.drawable.road_7, R.drawable.road_8, R.drawable.road_9,
            R.drawable.road_10, R.drawable.road_11, R.drawable.road_12, R.drawable.road_13, R.drawable.road_14,
            R.drawable.road_15, R.drawable.road_16, R.drawable.road_17, R.drawable.road_18, R.drawable.road_19,
            R.drawable.road_20,R.drawable.road_21,R.drawable.road_22,R.drawable.road_23,R.drawable.road_24,R.drawable.road_25};
    private int mCurIndex = 0;
    private boolean mShowGray = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Example of a call to a native method
        //TextView tv = findViewById(R.id.sample_text);
        //tv.setText(stringFromJNI());
        mImg = findViewById(R.id.img);
        mBtn1 = findViewById(R.id.btn_1);
        mBtnGray = findViewById(R.id.btn_gray);
        mBtnPre= findViewById(R.id.btn_pre);
        mBtnNext = findViewById(R.id.btn_next);
        mBtn1.setOnClickListener(this);
        mBtnGray.setOnClickListener(this);
        mBtnPre.setOnClickListener(this);
        mBtnNext.setOnClickListener(this);

        mHoughSB = findViewById(R.id.hough_value);
        mHoughSB.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                mHoughValue = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                updateImageView(mCurIndex);
            }
        });
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    //public native String stringFromJNI();
    //获得Canny边缘
    native void getEdge(Object bitmap, int hough, boolean showGray);

    @Override
    public void onClick(View v) {
        if(v.getId() == R.id.btn_1) {
            mCurIndex = 0;
        } else if(v.getId() == R.id.btn_gray) {
            mShowGray = !mShowGray;
            if(mShowGray){
                mBtnGray.setText(R.string.show_raw);
            } else {
                mBtnGray.setText(R.string.show_gray);
            }
        } else if(v.getId() == R.id.btn_pre) {
            --mCurIndex;
            if(mCurIndex < 1) {
                mCurIndex = mResId.length -1;
            }
        } else {
            ++mCurIndex;
            if(mCurIndex == mResId.length) {
                mCurIndex = 1;
            }
        }
        synchronized (this) {
            new Thread() {
                public void run() {
                    updateImageView(mCurIndex);
                }
            }.start();
        }
    }

    private void updateImageView(final int index){
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inSampleSize = 2;
        Bitmap bitmap = BitmapFactory.decodeResource(getResources(), mResId[index], options);
        getEdge(bitmap, mHoughValue, mShowGray);
        final Bitmap edgeBmp = bitmap;
        runOnUiThread(new Runnable() {
            public void run() {
                mImg.setImageBitmap(edgeBmp);
            }
        });
    }
}
