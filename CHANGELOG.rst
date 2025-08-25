^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package foxglove_compressed_video_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2025-08-25)
------------------
* fix param dump bug by changing decoder param sep from . to \_
* Contributors: Bernd Pfrommer

3.0.0 (2025-08-08)
------------------
* adjust to new image_transport and encoder/decoder API, added tests
  * adapt to new ffmpeg_encoder_decoder API
  * adjustments for new image_transport API
  * added tests
  * rename "map" parameter to "decoders"
  * parameters no longer have "." prefix
  * deal with humble brokenness where base_topic is not prefixed with namespace, but namespace is removed
  * package splitting functions into utilities file
* Contributors: Bernd Pfrommer

1.0.3 (2025-05-26)
------------------
* avoid ament_target_dependencies
* Add Constant Rate Factor,  Co-authored-by: Angsa Deployment Team <team@angsa-robotics.com>
* Update README.md
* Contributors: Bernd Pfrommer, Tony Najjar

1.0.2 (2025-03-30)
------------------
* Merge branch 'master' into release
* Fix segfault: the publish_fn address passed to publish() cannot be cached!
* Contributors: Michal Sojka

1.0.1 (2024-11-18)
------------------
* Fill empty message fields (`#1 <https://github.com/ros-misc-utilities/foxglove_compressed_video_transport/issues/1>`_)
  * Fix missing timestamp and frame_id in subscriber and publisher
* Contributors: Bernd Pfrommer, eLEcTRiCZiTy

1.0.0 (2024-09-16)
------------------
* initial commit
* Contributors: Bernd Pfrommer
